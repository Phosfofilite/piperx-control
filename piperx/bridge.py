"""Linux JSON-lines bridge. ConnectPort initialization is deliberately passive."""
import contextlib
import json
import math
import sys
import threading
import time

# Piper X preset from installed AgileX piper_param/constants.py (degrees).
# J6 reduced to ±170° after querying the actual motor configuration on this robot.
LIMITS = [(-150, 150), (0, 180), (-170, 0), (-89, 89), (-89, 89), (-170, 170)]

class Robot:
    def __init__(self, protocol="V3", channel="can0"):
        import fcntl
        import can
        import piper_sdk
        self.lock = open('/tmp/operation_ai_piper.lock', 'w')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.protocol = protocol
        cls = getattr(piper_sdk, 'C_PiperInterface_' + protocol)
        # Preserve actual feedback; application validates targets without clamping.
        self.sdk = cls(can_name=channel, start_sdk_joint_limit=False)
        self.sdk.ConnectPort(piper_init=False)
        self.bus = can.Bus(channel=channel, interface='socketcan', receive_own_messages=False)
        self.seen = {}
        self.guard = threading.RLock()
        self.armed_until = 0
        self.active = False
        self.closed = False
        self.last_error = None
        self.live_axis = None
        self.live_direction = 0
        self.live_deadline = 0
        self.live_target = None
        self.grip_commanded = False
        self.target_joints = None
        self.target_gripper = None
        self.target_deadline = 0
        self.target_result = None
        self.target_progress = None
        self.speed_deg_s = 2.0
        self.acceleration_deg_s2 = 4.0
        self.gripper_speed_mm_s = 5.0
        self.gripper_effort_nm = 1.0
        self.current_speed = 0.0
        self.current_gripper_speed = 0.0
        threading.Thread(target=self.monitor, daemon=True).start()
        threading.Thread(target=self.control_loop, daemon=True).start()

    def monitor(self):
        while not self.closed:
            try:
                msg = self.bus.recv(.05)
                with self.guard:
                    if msg and not msg.is_error_frame and not msg.is_extended_id and len(msg.data) == 8:
                        self.seen[msg.arbitration_id] = time.monotonic()
                    if self.active:
                        s = self.state()
                        if (time.monotonic() > self.armed_until or not s['fresh'] or
                            s['mode'] not in (0, 1) or s['fault'] or s['error_code']):
                            self.stop()
            except Exception as exc:
                self.last_error = str(exc)
                with self.guard:
                    if self.active:
                        try:
                            self.stop()
                        except Exception:
                            self.active = False
                time.sleep(.1)

    def fresh(self):
        # Each joint pair, status and six motor feedback messages must be current.
        ids = [0x2A1, 0x2A5, 0x2A6, 0x2A7] + list(range(0x261, 0x267))
        return all(time.monotonic() - self.seen.get(i, 0) < .5 for i in ids)

    def state(self):
        with self.guard:
            j = self.sdk.GetArmJointMsgs().joint_state
            status = self.sdk.GetArmStatus().arm_status
            grip = (self.sdk.GetArmGripperMsgs_V3() if self.protocol == 'V3' else self.sdk.GetArmGripperMsgs()).gripper_state
            grip_mode = int(getattr(grip, 'mode', 0))
            grip_value = getattr(grip, 'grippers_val', getattr(grip, 'grippers_angle', 0))/1000
            return dict(joints=[getattr(j, f'joint_{i}') / 1000 for i in range(1, 7)],
                        gripper_mm=grip_value if grip_mode == 0 else None,
                        gripper_mode=grip_mode, gripper_fault=int(grip.status_code)&63,
                        gripper_enabled=bool(int(grip.status_code)&64),
                        gripper_status_code=int(grip.status_code),
                        gripper_effort_nm=grip.grippers_effort/1000,
                        gripper_fresh=time.monotonic()-self.seen.get(0x2A8,0)<.5,
                        live_axis=self.live_axis,
                        target_joints=self.target_joints, target_gripper_mm=self.target_gripper,
                        moving_to_target=self.target_joints is not None or self.target_gripper is not None,
                        target_result=self.target_result,
                        settings=self.settings(),
                        fresh=self.fresh(), mode=int(status.ctrl_mode), fault=int(status.arm_status),
                        teach_status=int(status.teach_status),
                        error_code=int(status.err_code), enabled=list(self.sdk.GetArmEnableStatus()),
                        armed=self.active, seconds_left=max(0, self.armed_until-time.monotonic()),
                        firmware=str(self.sdk.GetPiperFirmwareVersion()), error=self.last_error)

    def check(self, allow_ended_teach=False):
        s = self.state()
        if not s['fresh']:
            raise ValueError('Retours CAN absents ou périmés (> 0,5 s).')
        ended_teach=(allow_ended_teach and getattr(self,'protocol',None)=='V3' and
                     s['mode']==2 and s.get('teach_status')==2)
        if s['mode'] not in (0, 1) and not ended_teach:
            raise ValueError('Bras en mode apprentissage/autre : quitter ce mode sur le robot avant activation.')
        if s['fault'] or s['error_code']:
            raise ValueError('Défaut robot : consulter le diagnostic et le matériel.')
        return s

    def arm(self, confirmed=False):
        if confirmed is not True:
            raise ValueError('Confirmation explicite requise.')
        with self.guard:
            import re
            self.firmware()
            firmware = str(self.sdk.GetPiperFirmwareVersion())
            match = re.search(r'V(\d+)\.(\d+)-(\d+)', firmware)
            if not match:
                raise ValueError('Firmware inconnu : activation interdite.')
            expected = 'V3' if tuple(map(int, match.groups())) >= (1, 8, 8) else 'V2'
            if self.protocol != expected:
                raise ValueError(f'Firmware {firmware} : sélectionner {expected}.')
            initial = self.check(allow_ended_teach=True)
            if any(not lo <= v <= hi for v, (lo, hi) in zip(initial['joints'], LIMITS)):
                raise ValueError('Position actuelle hors limites Piper X : vérifier position et calibration avant activation.')
            self.armed_until = time.monotonic() + 15
            self.active = True
            try:
                # Replace a previous controller's target before changing control mode.
                self.sdk.JointCtrl(*(round(v*1000) for v in initial['joints']))
                self.sdk.MotionCtrl_2(1, 1, self.sdk_speed_percent(), 0)
                transition_deadline=time.monotonic()+.3
                while int(self.sdk.GetArmStatus().arm_status.ctrl_mode)!=1:
                    if time.monotonic()>transition_deadline:
                        raise TimeoutError('Passage au contrôle CAN non confirmé ; aucun déplacement demandé.')
                    time.sleep(.01)
                deadline = time.monotonic() + 2
                while not all(self.sdk.GetArmEnableStatus()):
                    self.check()
                    if time.monotonic() > deadline:
                        raise TimeoutError('Activation moteurs non confirmée.')
                    self.sdk.EnableArm(7, 2)
                    time.sleep(.05)
            except Exception:
                self.stop()
                raise
        return self.state()

    def move(self, joints):
        with self.guard:
            s = self.check()
            if not self.active or time.monotonic() >= self.armed_until or not all(s['enabled']):
                raise ValueError('Activer les commandes et les moteurs avant déplacement.')
            if not isinstance(joints, list) or len(joints) != 6:
                raise ValueError('Six angles en degrés requis.')
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in joints):
                raise ValueError('Angles numériques finis requis.')
            for i, (v, old, bounds) in enumerate(zip(joints, s['joints'], LIMITS)):
                if not bounds[0] <= v <= bounds[1] or abs(v-old) > 2:
                    raise ValueError(f'J{i+1} : limite dépassée ou déplacement > 2 degrés.')
            self.sdk.MotionCtrl_2(1, 1, self.sdk_speed_percent(), 0)
            self.sdk.JointCtrl(*(round(v*1000) for v in joints))
        return {'accepted': True, 'target': joints, 'note': 'Envoi uniquement, vérifier les angles réels.'}

    def stop(self):
        with self.guard:
            self.live_axis = None
            self.target_joints = None
            self.target_gripper = None
            self.target_result = 'stopped'
            self.active = False
            self.armed_until = 0
            self.sdk.MotionCtrl_1(1, 0, 0)
            if getattr(self,'grip_commanded',False):
                s=self.state()
                if s['gripper_fresh'] and s['gripper_mode']==0 and not s['gripper_fault']:
                    self.sdk.GripperCtrl(round(s['gripper_mm']*1000),round(self.gripper_effort_nm*1000),1,0)
                self.grip_commanded=False
        return {'stopped': True, 'note': 'Arrêt logiciel demandé, non certifié. Réarmement matériel nécessaire.'}

    def gripper(self, width_mm):
        with self.guard:
            s = self.check()
            if not self.active or time.monotonic() >= self.armed_until:
                raise ValueError('Autoriser les commandes avant de piloter la pince.')
            if not s['gripper_fresh'] or s['gripper_mode'] != 0 or s['gripper_fault']:
                raise ValueError('Pince sans retour frais, en mode angle ou en défaut.')
            if isinstance(width_mm,bool) or not isinstance(width_mm,(int,float)) or not math.isfinite(width_mm):
                raise ValueError('Ouverture numérique finie requise.')
            if not 0 <= width_mm <= 50 or abs(width_mm-s['gripper_mm'])>2+1e-6:
                raise ValueError('Pince : plage de test 0–50 mm, variation maximale 2 mm.')
            self.sdk.GripperCtrl(round(width_mm*1000), round(self.gripper_effort_nm*1000), 1, 0)
            self.grip_commanded=True
            return {'accepted':True, 'gripper_target_mm':width_mm}

    def live(self, axis=None, direction=0):
        with self.guard:
            if direction == 0:
                self.hold()
                return self.state()
            if isinstance(axis,bool) or not isinstance(axis,int) or axis not in range(1,8) or direction not in (-1,1):
                raise ValueError('Axe 1…6 ou 7 pour la pince, direction −1/+1.')
            s = self.check()
            if not self.active or time.monotonic() >= self.armed_until:
                raise ValueError('Autoriser les commandes avant de maintenir un bouton.')
            if axis != self.live_axis:
                self.hold()
                self.live_target = s['joints'][:] if axis <= 6 else s['gripper_mm']
                self.current_speed=0.0
                self.current_gripper_speed=0.0
            self.live_axis = axis
            self.live_direction = direction
            self.live_deadline = time.monotonic()+.3
            self.armed_until = time.monotonic()+15
            return self.state()

    def hold(self):
        with self.guard:
            axis = self.live_axis
            if self.target_joints is not None:
                axis=1
            elif self.target_gripper is not None:
                axis=7
            self.live_axis = None
            if self.target_joints is not None or self.target_gripper is not None:
                self.target_result='cancelled'
            self.target_joints=None
            self.target_gripper=None
            self.current_speed=0.0
            self.current_gripper_speed=0.0
            if axis and self.active:
                s = self.check()
                if axis <= 6:
                    self.move(s['joints'])
                else:
                    self.gripper(s['gripper_mm'])

    def goto(self, joints=None, width_mm=None):
        with self.guard:
            s=self.check()
            if not self.active or time.monotonic()>=self.armed_until or not all(s['enabled']):
                raise ValueError('Autoriser les commandes et activer les moteurs avant déplacement.')
            if (joints is None)==(width_mm is None):
                raise ValueError('Fournir six angles OU une ouverture de pince.')
            if joints is not None:
                if not isinstance(joints,list) or len(joints)!=6:
                    raise ValueError('Six angles requis.')
                for i,(v,(lo,hi)) in enumerate(zip(joints,LIMITS),1):
                    if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:
                        raise ValueError(f'J{i} : angle hors limites [{lo}, {hi}] degrés.')
            else:
                if not s['gripper_fresh'] or s['gripper_mode']!=0 or s['gripper_fault']:
                    raise ValueError('Retours pince invalides pour une commande en mm.')
                if isinstance(width_mm,bool) or not isinstance(width_mm,(int,float)) or not math.isfinite(width_mm) or not 0<=width_mm<=50:
                    raise ValueError('Ouverture de pince comprise entre 0 et 50 mm requise.')
            self.hold()
            self.target_joints=list(joints) if joints is not None else None
            self.target_gripper=width_mm
            self.target_result='moving'
            self.target_progress=None
            self.current_speed=0.0
            self.current_gripper_speed=0.0
            self.live_target=s['joints'][:] if joints is not None else s['gripper_mm']
            self.keepalive()
            return self.state()

    def keepalive(self):
        with self.guard:
            if self.target_joints is not None or self.target_gripper is not None:
                self.check()
                self.target_deadline=time.monotonic()+.3
                self.armed_until=time.monotonic()+15
            return self.state()

    def target_tick(self, dt):
        if time.monotonic()>=self.target_deadline:
            self.hold()
            self.target_result='heartbeat_expired'
            return
        s=self.check()
        joint_move=self.target_joints is not None
        actual=s['joints'] if joint_move else [s['gripper_mm']]
        desired=self.target_joints if joint_move else [self.target_gripper]
        if not joint_move and (actual[0] is None or not s['gripper_fresh'] or s['gripper_fault'] or s['gripper_mode']!=0):
            raise ValueError('Perte des retours de pince pendant le déplacement.')
        error=max(abs(a-b) for a,b in zip(actual,desired))
        tolerance=.1 if joint_move else .2
        if error<=tolerance:
            self.hold()
            self.target_result='reached'
            return
        now=time.monotonic()
        if self.target_progress is None or error<self.target_progress[0]-.03:
            self.target_progress=(error,now)
        elif now-self.target_progress[1]>3:
            self.hold()
            self.target_result='no_progress'
            return
        previous=self.live_target if joint_move else [self.live_target]
        speed=self.current_speed if joint_move else self.current_gripper_speed
        remaining=max(abs(d-p) for p,d in zip(previous,desired))
        fraction=min(1,speed*dt/remaining) if remaining else 1
        lead=.5 if joint_move else 2.0
        # One common interpolation factor keeps all selected joints on the same path.
        for p,a,d in zip(previous,actual,desired):
            if d!=p:
                allowed=(a+(lead if d>p else -lead)-p)/(d-p)
                fraction=min(fraction,max(0,allowed))
        planned=[p+(d-p)*fraction for p,d in zip(previous,desired)]
        if joint_move:
            self.move(planned)
            self.live_target=planned
        else:
            planned_width=max(0,min(50,planned[0]))
            self.gripper(planned_width)
            self.live_target=planned_width

    def control_tick(self, dt):
        with self.guard:
            moving=self.live_axis is not None or self.target_joints is not None or self.target_gripper is not None
            if moving:
                requested_speed=self.speed_deg_s
                if self.target_joints is not None and self.live_target is not None:
                    distance=max(abs(d-p) for p,d in zip(self.live_target,self.target_joints))
                    requested_speed=min(requested_speed,math.sqrt(2*self.acceleration_deg_s2*max(0,distance-.05)))
                amount=self.acceleration_deg_s2*dt
                self.current_speed+=max(-amount,min(amount,requested_speed-self.current_speed))
                grip_amount=self.gripper_speed_mm_s*2*dt
                self.current_gripper_speed+=max(-grip_amount,min(grip_amount,self.gripper_speed_mm_s-self.current_gripper_speed))
            if self.target_joints is not None or self.target_gripper is not None:
                self.target_tick(dt)
                return
            axis = self.live_axis
            if axis is None:
                return
            if time.monotonic() >= self.live_deadline:
                self.hold()
                return
            s = self.check()
            if axis <= 6:
                i = axis-1
                target = list(self.live_target)
                lo,hi = LIMITS[i]
                value = target[i]+self.live_direction*self.current_speed*dt
                target[i] = max(lo,min(hi,max(s['joints'][i]-.5,min(s['joints'][i]+.5,value))))
                self.move(target)
                self.live_target = target
            else:
                if s['gripper_mm'] is None:
                    raise ValueError('Pince en mode angle : contrôle en mm indisponible.')
                value = self.live_target+self.live_direction*self.current_gripper_speed*dt
                target = max(0,min(50,max(s['gripper_mm']-2,min(s['gripper_mm']+2,value))))
                self.gripper(target)
                self.live_target = target

    def control_loop(self):
        previous = time.monotonic()
        while not self.closed:
            time.sleep(.02)
            now = time.monotonic()
            try:
                self.control_tick(min(now-previous,.04))
            except Exception as exc:
                self.last_error = str(exc)
                try:
                    self.stop()
                except Exception:
                    pass
            previous = now

    def firmware(self):
        self.sdk.SearchPiperFirmwareVersion()
        time.sleep(.2)
        return self.state()

    def sdk_speed_percent(self):
        return max(1,min(20,math.ceil(self.speed_deg_s*2.5)))

    def settings(self):
        return dict(speed_deg_s=self.speed_deg_s, acceleration_deg_s2=self.acceleration_deg_s2,
                    gripper_speed_mm_s=self.gripper_speed_mm_s, gripper_effort_nm=self.gripper_effort_nm)

    def configure(self, speed_deg_s=None, acceleration_deg_s2=None,
                  gripper_speed_mm_s=None, gripper_effort_nm=None):
        values=dict(speed_deg_s=speed_deg_s,acceleration_deg_s2=acceleration_deg_s2,
                    gripper_speed_mm_s=gripper_speed_mm_s,gripper_effort_nm=gripper_effort_nm)
        bounds=dict(speed_deg_s=(.2,10),acceleration_deg_s2=(.5,20),gripper_speed_mm_s=(1,20),gripper_effort_nm=(.2,1))
        with self.guard:
            for name,value in values.items():
                lo,hi=bounds[name]
                if value is not None and (isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi):
                    raise ValueError(f'{name} doit être compris entre {lo} et {hi}.')
            for name,value in values.items():
                if value is not None:
                    setattr(self,name,float(value))
            return self.state()

    def close(self):
        if self.active:
            self.stop()
        self.closed = True
        self.bus.shutdown()
        self.sdk.DisconnectPort()
        self.lock.close()

def main():
    output = sys.stdout
    robot = None
    for line in sys.stdin:
        try:
            request = json.loads(line)
            # Keep third-party SDK messages out of the RPC stream.
            with contextlib.redirect_stdout(sys.stderr):
                op = request['op']
                args = request.get('args', {})
                if op == 'connect':
                    if robot:
                        raise ValueError('Déjà connecté.')
                    robot = Robot(**args)
                    result = robot.state()
                elif op in ('state', 'arm', 'move', 'stop', 'firmware', 'gripper', 'live', 'goto', 'keepalive','configure') and robot:
                    result = getattr(robot, op)(**args)
                else:
                    raise ValueError('Commande inconnue ou connexion absente.')
            response = {'ok': True, 'result': result}
        except Exception as exc:
            response = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
        output.write(json.dumps(response, allow_nan=False) + '\n')
        output.flush()
    if robot:
        robot.close()

if __name__ == '__main__':
    main()
