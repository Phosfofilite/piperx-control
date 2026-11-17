"""Console Python de saisie d'angles. Simulation par défaut ; --real pour le matériel."""
import argparse
from piperx import Piper

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--real',action='store_true')
    args=parser.parse_args()
    with Piper(simulation=not args.real) as bras:
        print('BRAS RÉEL' if args.real else 'SIMULATION')
        print('Commandes : etat | autoriser | j1 20 | j3 -30 | pince 15 | pause | stop | quitter')
        while True:
            try:
                command=input('piperx> ').strip().lower().replace(',','.').split()
                if not command:
                    continue
                op=command[0]
                if op in ('quitter','exit'):
                    break
                if op=='etat':
                    print(bras.state())
                elif op=='autoriser':
                    confirmed=not args.real or input('Zone dégagée et arrêt matériel accessible ? Taper OUI : ')=='OUI'
                    print(bras.arm(confirmed=confirmed))
                elif op=='pause':
                    print(bras.hold())
                elif op=='stop':
                    print(bras.stop())
                elif op.startswith('j') and len(command)==2:
                    print(bras.set_joint_angle(int(op[1:]),float(command[1])))
                elif op=='pince' and len(command)==2:
                    print(bras.set_gripper_width(float(command[1])))
                else:
                    print('Commande inconnue.')
            except KeyboardInterrupt:
                bras.hold()
                print('\nCible interrompue. Taper quitter pour fermer la console.')
            except EOFError:
                break
            except Exception as exc:
                print(type(exc).__name__+': '+str(exc))

if __name__=='__main__':
    main()
