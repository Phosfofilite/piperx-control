"""Windows/Linux client usable by scripts and future AI controllers."""
import json
import pathlib
import platform
import queue
import subprocess
import threading

class Piper:
    def __init__(self, simulation=True, protocol='V3', distro='Ubuntu-24.04',
                 channel='can0', wsl_python='/opt/piperx/.venv/bin/python'):
        self.simulation = simulation
        self.process = None
        self.lock = threading.Lock()
        self.q = [0., 45., -45., 0., 0., 0.]
        self.active = False
        self.armed_until = 0
        self.grip = 10.
        self.live_axis = None
        self.live_direction = 0
        self.live_deadline = 0
        self.target_joints = None
        self.target_gripper = None
        self.target_deadline = 0
        self.target_result = None
        self.speed_deg_s=2.0
        self.acceleration_deg_s2=4.0
        self.gripper_speed_mm_s=5.0
        self.gripper_effort_nm=1.0
        self.current_speed=0.0
        self.current_gripper_speed=0.0
        import time
        self.last_sim_update = time.monotonic()
        if not simulation:
            bridge = pathlib.Path(__file__).with_name('bridge.py').resolve()
            if platform.system() == 'Windows':
                path = '/mnt/' + bridge.drive[0].lower() + str(bridge)[2:].replace('\\', '/')
                cmd = ['wsl', '-d', distro, '--', wsl_python, '-u', path]
            else:
                import sys
                cmd = [sys.executable, '-u', str(bridge)]
            self.process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.DEVNULL, text=True, encoding='utf-8',
                                            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            self.responses = queue.Queue()
            threading.Thread(target=self._reader, daemon=True).start()
            try:
                self._call('connect', protocol=protocol, channel=channel)
            except Exception:
                self.close()
                raise

    def _reader(self):
        for line in self.process.stdout:
            self.responses.put(line)
        self.responses.put('')

    def _call(self, op, **args):
        with self.lock:
            if self.process.poll() is not None:
                raise RuntimeError('Passerelle WSL arrêtée. Vérifier WSL, can0 et le SDK.')
            self.process.stdin.write(json.dumps({'op': op, 'args': args})+'\n')
            self.process.stdin.flush()
            try:
                line = self.responses.get(timeout=20 if op == 'connect' else 6)
            except queue.Empty:
                self.process.kill()
                raise TimeoutError('Passerelle sans réponse ; utiliser l’arrêt matériel si nécessaire.')
            if not line:
                raise RuntimeError('Passerelle fermée : vérifier can0 et le SDK installé.')
            response = json.loads(line)
            if not response['ok']:
                raise RuntimeError(response['error'])
            return response['result']

    def state(self):
        if self.simulation:
            import time
            from .bridge import LIMITS
            now = time.monotonic()
            targeting=self.target_joints is not None or self.target_gripper is not None
            deadline=self.target_deadline if targeting else self.live_deadline
            dt = max(0,min(now,deadline,self.armed_until)-self.last_sim_update)
            if self.active and (targeting or self.live_axis):
                import math
                requested_speed=self.speed_deg_s
                if self.target_joints is not None:
                    distance=max(abs(d-a) for a,d in zip(self.q,self.target_joints))
                    requested_speed=min(requested_speed,math.sqrt(2*self.acceleration_deg_s2*max(0,distance-.05)))
                amount=self.acceleration_deg_s2*dt
                self.current_speed+=max(-amount,min(amount,requested_speed-self.current_speed))
                amount=self.gripper_speed_mm_s*2*dt
                self.current_gripper_speed+=max(-amount,min(amount,self.gripper_speed_mm_s-self.current_gripper_speed))
            if self.active and targeting:
                if self.target_joints is not None:
                    distance=max(abs(d-a) for a,d in zip(self.q,self.target_joints))
                    fraction=min(1,self.current_speed*dt/distance) if distance else 1
                    self.q=[a+(d-a)*fraction for a,d in zip(self.q,self.target_joints)]
                    reached=max(abs(a-d) for a,d in zip(self.q,self.target_joints))<=.1
                else:
                    self.grip+=max(-self.current_gripper_speed*dt,min(self.current_gripper_speed*dt,self.target_gripper-self.grip))
                    reached=abs(self.grip-self.target_gripper)<=.2
                if reached or now>=self.target_deadline:
                    self.target_joints=None
                    self.target_gripper=None
                    self.target_result='reached' if reached else 'heartbeat_expired'
            if self.active and self.live_axis:
                if self.live_axis <= 6:
                    i=self.live_axis-1
                    lo,hi=LIMITS[i]
                    self.q[i]=max(lo,min(hi,self.q[i]+self.live_direction*self.current_speed*dt))
                else:
                    self.grip=max(0,min(50,self.grip+self.live_direction*self.current_gripper_speed*dt))
            self.last_sim_update = now
            if now >= self.live_deadline:
                self.live_axis = None
            if time.monotonic() > self.armed_until:
                self.active = False
            return dict(joints=self.q[:], fresh=True, mode=1, fault=0, error_code=0,
                        teach_status=2,
                        gripper_mm=self.grip, gripper_mode=0, gripper_fault=0, gripper_fresh=True,
                        gripper_enabled=self.active,gripper_status_code=64 if self.active else 0,gripper_effort_nm=0,
                        live_axis=self.live_axis,
                        target_joints=self.target_joints, target_gripper_mm=self.target_gripper,
                        moving_to_target=self.target_joints is not None or self.target_gripper is not None,
                        target_result=self.target_result,
                        settings=self.settings(),
                        enabled=[self.active]*6, armed=self.active, seconds_left=max(0,self.armed_until-time.monotonic()),
                        firmware='SIMULATION', error=None)
        return self._call('state')

    def arm(self, confirmed=False):
        if confirmed is not True:
            raise ValueError('Confirmation explicite requise.')
        if self.simulation:
            import time
            self.active = True
            self.armed_until = time.monotonic()+15
            return self.state()
        return self._call('arm', confirmed=confirmed)

    def move_joints(self, joints):
        if self.simulation:
            import math
            from .bridge import LIMITS
            self.state()
            if not self.active:
                raise ValueError('Activer les commandes.')
            if len(joints) != 6 or any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) for v in joints):
                raise ValueError('Six angles finis requis.')
            if any(not lo <= v <= hi or abs(v-old)>2 for v,old,(lo,hi) in zip(joints,self.q,LIMITS)):
                raise ValueError('Limite dépassée ou déplacement > 2 degrés.')
            self.q = list(joints)
            return {'accepted': True, 'target': self.q}
        return self._call('move', joints=list(joints))

    def jog(self, joint, delta_deg):
        if isinstance(joint, bool) or not isinstance(joint, int) or not 1 <= joint <= 6:
            raise ValueError('Articulation de 1 à 6.')
        q = self.state()['joints']
        q[joint-1] += delta_deg
        return self.move_joints(q)

    def stop(self):
        if self.simulation:
            self.state()
            self.live_axis = None
            self.target_joints=None
            self.target_gripper=None
            self.target_result='stopped'
            self.active = False
            self.armed_until = 0
            return self.state()
        return self._call('stop')

    def set_gripper(self, width_mm):
        if self.simulation:
            import math
            s=self.state()
            if not s['armed'] or isinstance(width_mm,bool) or not isinstance(width_mm,(int,float)) or not math.isfinite(width_mm):
                raise ValueError('Autorisation et ouverture numérique finie requises.')
            if not 0 <= width_mm <= 50 or abs(width_mm-self.grip)>2:
                raise ValueError('Pince : 0–50 mm et variation maximale 2 mm.')
            self.grip=width_mm
            return {'accepted':True,'gripper_target_mm':width_mm}
        return self._call('gripper',width_mm=width_mm)

    def live(self, axis=None, direction=0):
        if self.simulation:
            import time
            s=self.state()
            if direction == 0:
                self.live_axis=None
                self.current_speed=0.0
                self.current_gripper_speed=0.0
                if self.target_joints is not None or self.target_gripper is not None:
                    self.target_result='cancelled'
                self.target_joints=None
                self.target_gripper=None
            else:
                if not s['armed'] or isinstance(axis,bool) or not isinstance(axis,int) or axis not in range(1,8) or direction not in (-1,1):
                    raise ValueError('Autorisation, axe 1…7 et direction −1/+1 requis.')
                if axis!=self.live_axis:
                    self.current_speed=0.0
                    self.current_gripper_speed=0.0
                self.live_axis=axis
                self.target_joints=None
                self.target_gripper=None
                self.live_direction=direction
                self.live_deadline=time.monotonic()+.3
                self.armed_until=time.monotonic()+15
            return self.state()
        return self._call('live',axis=axis,direction=direction)

    def hold(self):
        return self.live(direction=0)

    def _start_target(self, joints=None, width_mm=None):
        if not self.simulation:
            return self._call('goto',joints=joints,width_mm=width_mm)
        from .bridge import LIMITS
        import math,time
        s=self.state()
        if not s['armed']:
            raise ValueError('Autoriser les commandes avant déplacement.')
        if (joints is None)==(width_mm is None):
            raise ValueError('Fournir six angles OU une ouverture de pince.')
        if joints is not None:
            if len(joints)!=6 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi for v,(lo,hi) in zip(joints,LIMITS)):
                raise ValueError('Six angles finis dans les limites articulaires requis.')
        elif isinstance(width_mm,bool) or not isinstance(width_mm,(int,float)) or not math.isfinite(width_mm) or not 0<=width_mm<=50:
            raise ValueError('Ouverture de pince comprise entre 0 et 50 mm requise.')
        self.hold()
        self.target_joints=list(joints) if joints is not None else None
        self.target_gripper=width_mm
        self.target_result='moving'
        self.target_deadline=time.monotonic()+.3
        self.armed_until=time.monotonic()+15
        return self.state()

    def keepalive(self):
        """Renew an asynchronous target. Call at least every 100 ms while moving."""
        if self.simulation:
            import time
            s=self.state()
            if s['moving_to_target']:
                self.target_deadline=time.monotonic()+.3
                self.armed_until=time.monotonic()+15
            return self.state()
        return self._call('keepalive')

    def wait_until_reached(self, timeout=120):
        """Wait for measured arrival, renewing the target; hold on timeout/error."""
        import time,math
        if isinstance(timeout,bool) or not isinstance(timeout,(int,float)) or not math.isfinite(timeout) or timeout<=0:
            raise ValueError('Délai positif fini requis.')
        deadline=time.monotonic()+timeout
        try:
            while True:
                s=self.keepalive()
                if not s['moving_to_target']:
                    if s.get('target_result')=='reached':
                        return s
                    raise RuntimeError(f'Déplacement interrompu : {s.get("target_result")} ; {s.get("error")}')
                if time.monotonic()>=deadline:
                    raise TimeoutError('Cible non atteinte dans le délai imparti.')
                time.sleep(.08)
        except BaseException:
            try:
                self.hold()
            except Exception:
                pass
            raise

    @staticmethod
    def _validate_wait_timeout(wait, timeout):
        import math
        if not isinstance(wait,bool):
            raise ValueError('wait doit être True ou False.')
        if wait and (isinstance(timeout,bool) or not isinstance(timeout,(int,float)) or not math.isfinite(timeout) or timeout<=0):
            raise ValueError('Délai positif fini requis avant de démarrer un déplacement.')

    def set_joint_angle(self, joint, angle_deg, *, wait=True, timeout=120):
        """Move one joint (1..6) to an absolute angle in degrees at up to 2°/s."""
        if isinstance(joint,bool) or not isinstance(joint,int) or not 1<=joint<=6:
            raise ValueError('Articulation de 1 à 6.')
        q=self.state()['joints']
        q[joint-1]=angle_deg
        return self.set_joint_angles(q,wait=wait,timeout=timeout)

    def set_joint_angles(self, angles_deg, *, wait=True, timeout=120):
        """Move to six absolute angles. wait=False requires keepalive() calls."""
        self._validate_wait_timeout(wait,timeout)
        result=self._start_target(joints=list(angles_deg))
        return self.wait_until_reached(timeout) if wait else result

    def set_gripper_width(self, width_mm, *, wait=True, timeout=30):
        """Move the gripper to an absolute opening in mm, within the 0..50 mm test range."""
        self._validate_wait_timeout(wait,timeout)
        result=self._start_target(width_mm=width_mm)
        return self.wait_until_reached(timeout) if wait else result

    def firmware(self):
        return self.state() if self.simulation else self._call('firmware')

    def settings(self):
        if not self.simulation:
            return self.state()['settings']
        return dict(speed_deg_s=self.speed_deg_s,acceleration_deg_s2=self.acceleration_deg_s2,
                    gripper_speed_mm_s=self.gripper_speed_mm_s,gripper_effort_nm=self.gripper_effort_nm)

    def configure(self, *, speed_deg_s=None, acceleration_deg_s2=None,
                  gripper_speed_mm_s=None, gripper_effort_nm=None):
        """Set arm speed/acceleration and gripper speed/torque within the test limits."""
        args=dict(speed_deg_s=speed_deg_s,acceleration_deg_s2=acceleration_deg_s2,
                  gripper_speed_mm_s=gripper_speed_mm_s,gripper_effort_nm=gripper_effort_nm)
        if not self.simulation:
            return self._call('configure',**args)
        import math
        bounds=dict(speed_deg_s=(.2,10),acceleration_deg_s2=(.5,20),gripper_speed_mm_s=(1,20),gripper_effort_nm=(.2,1))
        for name,value in args.items():
            lo,hi=bounds[name]
            if value is not None and (isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi):
                raise ValueError(f'{name} doit être compris entre {lo} et {hi}.')
        self.state()
        for name,value in args.items():
            if value is not None:
                setattr(self,name,float(value))
        return self.state()

    def close(self):
        if self.process:
            with self.lock:
                if self.process.poll() is None:
                    self.process.stdin.close()
                    try:
                        self.process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                self.process = None
        self.active = False

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
