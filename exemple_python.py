"""Simulation par défaut. Fermer l’application avant un accès réel exclusif."""
from piper_client import Piper

with Piper(simulation=True) as bras:
    print(bras.state())
    bras.arm(confirmed=True)
    bras.jog(joint=1, delta_deg=0.5)
    print(bras.state()['joints'])
    bras.stop()

# Accès réel : Piper(simulation=False, protocol='V3').
# Vérifier le firmware : V3 pour >= 1.8-8.
# Ne pas armer avant validation humaine de la zone et du mode robot.
