"""Native desktop console, no packages needed on Windows."""
import concurrent.futures
import json
import pathlib
import queue
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from piperx import Piper

# Avoid Windows bitmap scaling that can clip the control panel on high-DPI displays.
try:
    import ctypes
    ctypes.windll.shcore.SetProcessDpiAwareness(1)
except (AttributeError, OSError):
    pass

BG, PANEL, TEXT, MUTED, ACCENT = '#101820', '#1c2935', '#edf4fa', '#9fb4c4', '#4ee1aa'

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Opération AI • PiperX Control')
        self.geometry(f'{min(1240,self.winfo_screenwidth()-60)}x{min(1050,self.winfo_screenheight()-100)}')
        self.minsize(1100, 950)
        self.configure(bg=BG)
        self.robot = None
        self.snapshot = None
        self.busy = False
        self.polling = False
        self.pending = None
        self.stopping = False
        self.held_axis = None
        self.held_direction = 0
        self.target_running = False
        self.inputs_loaded = False
        self.last_target_result = None
        self.events = queue.Queue()
        self.worker = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.protocol('WM_DELETE_WINDOW', self.quit_app)
        style = ttk.Style(self)
        style.theme_use('clam')
        style.configure('.', background=PANEL, foreground=TEXT, font=('Segoe UI', 11))
        style.configure('TButton', padding=(14, 9))
        style.map('TButton', background=[('disabled', '#23313d'), ('active', '#355267'), ('!disabled', '#293e50')], foreground=[('disabled', '#718695'), ('!disabled', TEXT)])
        style.configure('TCombobox', fieldbackground=PANEL, foreground=TEXT)
        style.map('TCombobox', fieldbackground=[('readonly', '#293e50'), ('disabled', '#23313d')], foreground=[('disabled', MUTED), ('readonly', TEXT)], selectbackground=[('readonly', '#293e50')], selectforeground=[('readonly', TEXT)])
        style.configure('TCheckbutton', background=BG)
        self.label('OPÉRATION AI', 11, ACCENT).pack(anchor='w', padx=30, pady=(12, 0))
        self.label('PiperX Control', 24).pack(anchor='w', padx=30)
        self.label('6 axes + pince · Commandes continues ou cibles absolues · Positions mesurées en direct', 11, MUTED).pack(anchor='w', padx=30, pady=(0, 15))
        top = tk.Frame(self, bg=BG)
        top.pack(fill='x', padx=30)
        self.mode = tk.StringVar(value='Bras réel / WSL')
        self.version = tk.StringVar(value='V3')
        self.modebox = ttk.Combobox(top, textvariable=self.mode, values=['Simulation', 'Bras réel / WSL'], state='readonly', width=20)
        self.modebox.pack(side='left', padx=(0, 10))
        self.verbox = ttk.Combobox(top, textvariable=self.version, values=['V2', 'V3'], state='readonly', width=5)
        self.verbox.pack(side='left', padx=(0, 10))
        self.connect_btn = ttk.Button(top, text='Connecter en lecture', command=self.connect_robot)
        self.connect_btn.pack(side='left')
        self.indicator = self.label('DÉCONNECTÉ', 11, MUTED)
        self.indicator.pack(anchor='w', padx=30, pady=12)
        grid = tk.Frame(self, bg=PANEL)
        grid.pack(fill='x', padx=30)
        for col, title in enumerate(['ARTICULATION', 'ANGLE MESURÉ', 'MAINTENIR + / −', 'CIBLE ABSOLUE']):
            tk.Label(grid, text=title, bg=PANEL, fg=MUTED, font=('Segoe UI', 10)).grid(row=0, column=col, padx=20, pady=12, sticky='w')
        self.angles = []
        self.motion_buttons = []
        self.angle_inputs = []
        self.target_buttons = []
        for i, name in enumerate(['Base', 'Épaule', 'Coude', 'Poignet 1', 'Poignet 2', 'Poignet 3']):
            tk.Label(grid, text=f'J{i+1}  {name}', bg=PANEL, fg=TEXT, font=('Segoe UI', 12)).grid(row=i+1, column=0, padx=20, pady=7, sticky='w')
            angle = tk.Label(grid, text='— °', bg=PANEL, fg=ACCENT, font=('Consolas', 17))
            angle.grid(row=i+1, column=1, padx=20)
            self.angles.append(angle)
            row = tk.Frame(grid, bg=PANEL)
            row.grid(row=i+1, column=2, padx=20, pady=4)
            for sign, text in [(-1, '−'), (1, '+')]:
                b = ttk.Button(row, text=text, width=6)
                b.bind('<ButtonPress-1>',lambda e,j=i+1,d=sign:self.begin_hold(j,d))
                b.bind('<ButtonRelease-1>',self.end_hold)
                b.bind('<Leave>',self.end_hold)
                b.pack(side='left', padx=4)
                self.motion_buttons.append(b)
            targetrow=tk.Frame(grid,bg=PANEL)
            targetrow.grid(row=i+1,column=3,padx=16,pady=4)
            value=tk.StringVar(value='')
            self.angle_inputs.append(value)
            entry=tk.Entry(targetrow,textvariable=value,width=8,bg='#101820',fg=TEXT,insertbackground=TEXT,font=('Consolas',13),relief='flat')
            entry.pack(side='left',padx=(0,8),ipady=6)
            entry.bind('<Return>',lambda e,j=i+1:self.apply_angle(j))
            button=ttk.Button(targetrow,text='Aller',width=5,command=lambda j=i+1:self.apply_angle(j))
            button.pack(side='left')
            self.target_buttons.append(button)
        tk.Label(grid,text='Pince',bg=PANEL,fg=TEXT,font=('Segoe UI',12)).grid(row=7,column=0,padx=20,pady=7,sticky='w')
        self.gripper_label=tk.Label(grid,text='— mm',bg=PANEL,fg=ACCENT,font=('Consolas',17))
        self.gripper_label.grid(row=7,column=1,padx=20)
        griprow=tk.Frame(grid,bg=PANEL)
        griprow.grid(row=7,column=2,padx=20,pady=4)
        self.gripper_buttons=[]
        for direction,title in [(-1,'Fermer'),(1,'Ouvrir')]:
            b=ttk.Button(griprow,text=title,width=8)
            b.pack(side='left',padx=4)
            b.bind('<ButtonPress-1>',lambda e,d=direction:self.begin_hold(7,d))
            b.bind('<ButtonRelease-1>',self.end_hold)
            b.bind('<Leave>',self.end_hold)
            self.gripper_buttons.append(b)
        griptarget=tk.Frame(grid,bg=PANEL)
        griptarget.grid(row=7,column=3,padx=16,pady=4)
        self.gripper_input=tk.StringVar(value='')
        gripentry=tk.Entry(griptarget,textvariable=self.gripper_input,width=8,bg='#101820',fg=TEXT,insertbackground=TEXT,font=('Consolas',13),relief='flat')
        gripentry.pack(side='left',padx=(0,8),ipady=6)
        gripentry.bind('<Return>',lambda e:self.apply_gripper())
        self.grip_target_button=ttk.Button(griptarget,text='Aller',width=5,command=self.apply_gripper)
        self.grip_target_button.pack(side='left')
        self.bind('<FocusOut>',self.focus_lost)
        self.bind('<Escape>',self.end_hold)
        grid.grid_columnconfigure(2, weight=1)
        settings=tk.Frame(self,bg=BG)
        settings.pack(fill='x',padx=30,pady=8)
        self.setting_inputs={}
        for column,(key,title,initial,choices) in enumerate([
            ('speed_deg_s','Bras °/s','2',[.2,.5,1,2,4,6,8,10]),
            ('acceleration_deg_s2','Accélération °/s²','4',[.5,1,2,4,8,12,20]),
            ('gripper_speed_mm_s','Pince mm/s','5',[1,2,5,10,20]),
            ('gripper_effort_nm','Couple pince N·m','1',[.2,.5,.75,1])]):
            self.label_in(settings,title).grid(row=0,column=column*2,padx=(0,5))
            var=tk.StringVar(value=initial)
            self.setting_inputs[key]=var
            ttk.Combobox(settings,textvariable=var,values=choices,width=5,state='readonly').grid(row=0,column=column*2+1,padx=(0,10))
        ttk.Button(settings,text='Appliquer réglages',command=self.apply_settings).grid(row=0,column=8)
        controls = tk.Frame(self, bg=BG)
        controls.pack(fill='x', padx=30, pady=12)
        self.step = tk.StringVar(value='0.5')
        self.arm_btn = ttk.Button(controls, text='Autoriser 15 secondes', command=self.arm_robot)
        self.arm_btn.pack(side='left', padx=8)
        ttk.Button(controls,text='Interrompre la cible',command=self.cancel_target).pack(side='left',padx=8)
        self.all_targets_button=ttk.Button(controls,text='Envoyer les 6 angles',command=self.apply_all_angles)
        self.all_targets_button.pack(side='left',padx=8)
        tk.Button(controls, text='■  ARRÊT LOGICIEL', command=self.stop_robot, bg='#c74146', fg='white', font=('Segoe UI', 12, 'bold'), relief='flat', padx=20, pady=9).pack(side='right')
        self.speed_info=self.label('Consignes : bras 2°/s · accélération 4°/s² · pince 5 mm/s · couple pince 1 N·m',10,MUTED)
        self.speed_info.pack(anchor='w',padx=30)
        self.detail = self.label('Choisir V3 pour firmware ≥ 1.8-8. Vérifier le firmware avant activation réelle.', 10, MUTED)
        self.detail.pack(anchor='w', padx=30, pady=6)
        tools = tk.Frame(self, bg=BG)
        tools.pack(fill='x', padx=30, pady=6)
        ttk.Button(tools, text='Lire le firmware', command=lambda: self.action(lambda: self.robot.firmware()) if self.robot else None).pack(side='left')
        ttk.Button(tools, text='Exporter la position', command=self.export).pack(side='left', padx=8)
        ttk.Button(tools, text='Déconnecter', command=self.disconnect).pack(side='left')
        ttk.Button(tools,text='Charger les angles mesurés',command=self.load_inputs).pack(side='left',padx=8)
        self.log = tk.Text(self, height=5, bg='#0b1218', fg=MUTED, insertbackground=TEXT, relief='flat', font=('Consolas', 10))
        self.log.pack(fill='both', expand=True, padx=30, pady=(8, 18))
        self.write('Lecture seule au démarrage. L’arrêt logiciel ne remplace pas l’arrêt matériel.')
        self.after(15, self.pump)
        self.after(100, self.poll)
        self.refresh_buttons()

    def label(self, text, size=12, color=TEXT):
        return tk.Label(self, text=text, bg=BG, fg=color, font=('Segoe UI', size), anchor='w')

    def label_in(self, parent, text):
        return tk.Label(parent, text=text, bg=BG, fg=TEXT, font=('Segoe UI', 11))

    def write(self, text):
        self.log.insert('end', text+'\n')
        self.log.see('end')

    def action(self, fn, callback=None, quiet=False):
        if self.busy:
            if not quiet:
                # Keep the latest user intent while a telemetry read finishes.
                self.pending = (fn, callback, quiet)
            return
        self.busy = True
        self.polling = quiet
        def run():
            try:
                self.events.put((True, fn(), callback, quiet))
            except Exception as exc:
                self.events.put((False, str(exc), None, quiet))
        self.worker.submit(run)
        self.refresh_buttons()

    def pump(self):
        try:
            ok, result, callback, quiet = self.events.get_nowait()
            self.busy = False
            self.polling = False
            if ok:
                if callback:
                    callback(result)
                elif isinstance(result, dict) and 'joints' in result:
                    self.show_state(result)
                elif not quiet:
                    self.write(json.dumps(result, ensure_ascii=False))
            else:
                self.target_running=False
                self.held_axis=None
                self.pending=None
                self.write(result)
                self.snapshot = None
                self.indicator.config(text='ERREUR — commandes bloquées', fg='#ff8a80')
                if not quiet:
                    messagebox.showerror('PiperX', result)
            self.refresh_buttons()
            if self.pending:
                task = self.pending
                self.pending = None
                self.action(*task)
        except queue.Empty:
            pass
        self.after(15, self.pump)

    def refresh_buttons(self):
        available = (not self.busy or self.polling or self.held_axis is not None) and not self.stopping
        ready = self.robot and self.snapshot and self.snapshot['fresh'] and self.snapshot['armed'] and available
        for b in self.motion_buttons:
            b.configure(state='normal' if ready else 'disabled')
        for b in self.target_buttons:
            b.configure(state='normal' if ready else 'disabled')
        self.all_targets_button.configure(state='normal' if ready else 'disabled')
        gripready=ready and self.snapshot.get('gripper_fresh') and self.snapshot.get('gripper_mode')==0 and not self.snapshot.get('gripper_fault')
        for b in self.gripper_buttons:
            b.configure(state='normal' if gripready else 'disabled')
        self.grip_target_button.configure(state='normal' if gripready else 'disabled')
        self.arm_btn.configure(state='normal' if self.robot and available else 'disabled')
        self.connect_btn.configure(state='normal' if not self.robot and not self.busy else 'disabled')
        for b in (self.modebox, self.verbox):
            b.configure(state='disabled' if self.robot or self.busy else 'readonly')

    def connect_robot(self):
        simulation = self.mode.get() == 'Simulation'
        version = self.version.get()
        def connect():
            robot = Piper(simulation=simulation, protocol=version)
            return robot
        def done(robot):
            self.robot = robot
            self.inputs_loaded=False
            self.write('Connexion simulation.' if simulation else 'Connexion WSL réelle, aucune commande moteur envoyée.')
        self.action(connect, done)

    def show_state(self, s):
        self.snapshot = s
        self.target_running=s.get('moving_to_target',False)
        result=s.get('target_result')
        if result != self.last_target_result:
            messages={'reached':'Cible atteinte dans la tolérance de mesure.',
                      'no_progress':'Cible interrompue : absence de progression pendant 3 secondes.',
                      'heartbeat_expired':'Cible interrompue : renouvellement de commande perdu.',
                      'cancelled':'Déplacement vers la cible interrompu.'}
            if result in messages:
                self.write(messages[result])
            self.last_target_result=result
        if s['fresh'] and not self.inputs_loaded:
            self.load_inputs()
            self.inputs_loaded=True
        if s.get('error'):
            self.end_hold()
        for widget, v in zip(self.angles, s['joints']):
            widget.config(text=f'{v:+8.3f} °' if s['fresh'] else '— °')
        grip=s.get('gripper_mm')
        self.gripper_label.config(text=f'{grip:7.2f} mm' if grip is not None and s.get('gripper_fresh') else '— mm')
        config=s.get('settings',{})
        if config:
            self.speed_info.config(text=f'Consignes : bras {config["speed_deg_s"]:g}°/s · accélération {config["acceleration_deg_s2"]:g}°/s² · pince {config["gripper_speed_mm_s"]:g} mm/s · couple {config["gripper_effort_nm"]:g} N·m')
        mode = 'SIMULATION' if self.robot.simulation else 'BRAS RÉEL'
        text = 'VERS LA CIBLE' if self.target_running else ('COMMANDES AUTORISÉES' if s['armed'] else 'LECTURE SEULE')
        self.indicator.config(text=f'{mode}   •   {text}   •   CAN {"OK" if s["fresh"] else "PÉRIMÉ"}   •   {s["seconds_left"]:.0f} s', fg=ACCENT if s['fresh'] else '#ff8a80')
        teach=s.get('teach_status')
        teachtext='enregistrement terminé' if teach==2 else f'état apprentissage {teach}'
        griptext='activée' if s.get('gripper_enabled') else 'désactivée'
        self.detail.config(text=f'Mode {s["mode"]} | {teachtext} | Défaut {s["fault"]} | Pince {griptext}, défaut {s.get("gripper_fault",0)}, couple mesuré {s.get("gripper_effort_nm",0):.3f} N·m')

    def poll(self):
        if self.robot and not self.busy:
            if self.held_axis is not None:
                axis,direction=self.held_axis,self.held_direction
                self.action(lambda:self.robot.live(axis,direction),quiet=True)
            elif self.target_running:
                self.action(self.robot.keepalive,quiet=True)
            else:
                self.action(self.robot.state, quiet=True)
        self.after(100, self.poll)

    def begin_hold(self,axis,direction):
        if self.stopping or not self.robot or not self.snapshot or not self.snapshot['armed']:
            return 'break'
        self.held_axis=axis
        self.target_running=False
        self.held_direction=direction
        self.action(lambda:self.robot.live(axis,direction),quiet=True)
        return 'break'

    def end_hold(self,event=None):
        if self.held_axis is not None or self.target_running:
            self.held_axis=None
            self.target_running=False
            self.pending=None
            if self.robot:
                self.action(self.robot.hold,quiet=False)
        return 'break'

    def load_inputs(self):
        if self.snapshot and self.snapshot['fresh']:
            for var,value in zip(self.angle_inputs,self.snapshot['joints']):
                var.set(f'{value:.3f}')
            if self.snapshot.get('gripper_mm') is not None:
                self.gripper_input.set(f'{self.snapshot["gripper_mm"]:.2f}')

    def apply_angle(self,joint):
        if not self.robot or self.stopping:
            return 'break'
        try:
            value=float(self.angle_inputs[joint-1].get().strip().replace(',','.'))
        except ValueError:
            messagebox.showerror('Angle cible','Saisir un angle en degrés, par exemple 20 ou −15,5.')
            return 'break'
        self.held_axis=None
        self.target_running=True
        self.action(lambda:self.robot.set_joint_angle(joint,value,wait=False))
        return 'break'

    def apply_all_angles(self):
        if not self.robot or self.stopping:
            return
        try:
            values=[float(var.get().strip().replace(',','.')) for var in self.angle_inputs]
        except ValueError:
            messagebox.showerror('Cibles articulaires','Remplir les six champs avec des angles en degrés.')
            return
        self.held_axis=None
        self.target_running=True
        self.action(lambda:self.robot.set_joint_angles(values,wait=False))

    def apply_settings(self):
        if not self.robot:
            self.write('Connecter le pilote avant d’appliquer les réglages.')
            return
        try:
            values={key:float(var.get().replace(',','.')) for key,var in self.setting_inputs.items()}
        except ValueError:
            messagebox.showerror('Réglages','Les réglages doivent être numériques.')
            return
        self.action(lambda:self.robot.configure(**values))

    def apply_gripper(self):
        if not self.robot or self.stopping:
            return 'break'
        try:
            value=float(self.gripper_input.get().strip().replace(',','.'))
        except ValueError:
            messagebox.showerror('Cible pince','Saisir une ouverture en mm, entre 0 et 50.')
            return 'break'
        self.held_axis=None
        self.target_running=True
        self.action(lambda:self.robot.set_gripper_width(value,wait=False))
        return 'break'

    def cancel_target(self):
        self.held_axis=None
        self.target_running=False
        self.pending=None
        if self.robot:
            self.action(self.robot.hold)

    def focus_lost(self,event):
        self.after(1,lambda:self.end_hold() if self.focus_displayof() is None else None)

    def arm_robot(self):
        if not self.robot or self.stopping:
            return
        if not self.robot.simulation and not messagebox.askokcancel('Activation réelle des moteurs', 'Le bras peut réagir lors de l’activation.\nZone dégagée et arrêt matériel accessible ?\nFirmware et protocole V2/V3 vérifiés ?\n\nAutoriser les commandes à 5 % pour 15 secondes ?'):
            return
        self.action(lambda: self.robot.arm(confirmed=True))

    def jog(self, joint, sign):
        if self.stopping:
            return
        delta = sign*float(self.step.get())
        self.action(lambda: self.robot.jog(joint, delta))

    def stop_robot(self):
        if self.robot:
            self.held_axis=None
            self.target_running=False
            self.stopping = True
            self.refresh_buttons()
            if self.busy:
                self.write('Commande en cours : utiliser l’arrêt matériel si nécessaire.')
            self.pending = None
            def done(result):
                self.stopping = False
                self.write(json.dumps(result, ensure_ascii=False))
            self.action(self.robot.stop, done)

    def export(self):
        if not self.snapshot or not self.snapshot['fresh']:
            return
        path = filedialog.asksaveasfilename(defaultextension='.json', initialfile='position_piper.json')
        if path:
            pathlib.Path(path).write_text(json.dumps(self.snapshot, indent=2, ensure_ascii=False), encoding='utf-8')
            self.write('Position exportée : '+path)

    def disconnect(self):
        if not self.robot or not messagebox.askyesno('Confirmer la déconnexion','Déconnecter le bras ? Les commandes de cette session seront interrompues.'):
            return
        self.end_hold()
        if self.robot and not self.busy:
            def done(_):
                self.robot = None
                self.snapshot = None
                self.indicator.config(text='DÉCONNECTÉ', fg=MUTED)
            self.action(self.robot.close, done)

    def quit_app(self):
        if not messagebox.askyesno('Confirmer la fermeture','Fermer PiperX Control ?\nLa connexion et les commandes de cette session seront arrêtées.'):
            return
        self._finish_close()

    def _finish_close(self):
        self.end_hold()
        if self.busy:
            self.after(50,self._finish_close)
            return
        if self.robot:
            self.robot.close()
        self.worker.shutdown(wait=False)
        self.destroy()

if __name__ == '__main__':
    import sys
    application=App()
    if '--connect' in sys.argv:
        application.after(200,application.connect_robot)
    application.mainloop()
