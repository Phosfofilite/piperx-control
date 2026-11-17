"""Six axes + pince : exemple interactif simulé. Aucune activation réelle."""
import time
from piper_client import Piper

with Piper(simulation=True) as bras:
    bras.arm(confirmed=True)
    for axis, label in [(1,'J1'), (7,'pince')]:
        print('Déplacement de',label)
        try:
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                bras.live(axis=axis,direction=1)
                print(bras.state()['joints'],bras.state()['gripper_mm'])
                time.sleep(.1)
        finally:
            bras.hold()
    bras.stop()
