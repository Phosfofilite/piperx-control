"""Exemple de bibliothèque, simulation par défaut. Installer : py -m pip install ."""
from piperx import Piper

with Piper(simulation=True) as bras:
    bras.arm(confirmed=True)
    bras.set_joint_angle(1, 2.0)       # Angle ABSOLU de J1, en degrés ; attend l'arrivée
    bras.set_gripper_width(15.0)      # Ouverture ABSOLUE en mm ; attend l'arrivée
    print('Angles mesurés :', bras.state()['joints'])
    print('Pince mesurée :', bras.state()['gripper_mm'])
    bras.hold()

# Pour le matériel : Piper(simulation=False).
# Fermer d'abord la connexion de l'application pour libérer le pilote exclusif.
