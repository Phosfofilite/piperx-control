# PiperX Control

Outil Python de pilotage local d’un bras **Piper X à six articulations et pince** : interface de bureau Tkinter, bibliothèque utilisable depuis des scripts, simulation sans matériel et passerelle CAN sous Linux/WSL.

Ce prototype explore la séparation entre interface, API de commande et validation des états du robot. Il est documenté et préparé avec assistance IA. Ce n’est pas un logiciel de sécurité certifié.

## Fonctionnalités

- Lecture des angles, de l’ouverture de pince, des défauts et du firmware.
- Cibles articulaires absolues, commandes relatives et commandes continues.
- Progression des consignes avec paramètres de vitesse et d’accélération.
- Confirmation d’activation, autorisation temporaire et surveillance du renouvellement des commandes.
- Simulation et tests avec pilote factice, sans matériel.

## Essayer sans robot

Python 3.10 ou supérieur est requis ; Tkinter est nécessaire pour l’interface.

```sh
python -m unittest discover -s . -p test_control.py -v
python console_python.py
```

La console démarre en simulation. Pour l’interface : `python app.py`, puis sélectionner **Simulation** avant connexion. L’interface propose aussi un mode matériel ; le simple lancement ne constitue pas un essai matériel.

Pour installer la bibliothèque :

```sh
python -m pip install .
```

```python
from piperx import Piper

with Piper(simulation=True) as robot:
    print(robot.state())
    robot.arm(confirmed=True)
    robot.set_joint_angle(1, 1.0)
    robot.hold()
    robot.stop()
```

## Architecture

```text
app.py / console_python.py / scripts
                 |
           piperx.client.Piper
            /                   simulation       JSON / sous-processus
                             |
                       piperx.bridge
                             |
                       SDK Piper / CAN
```

`piperx/client.py` expose l’API ; `piperx/bridge.py` gère les contrôles et la liaison SDK. Les fichiers `piper_client.py` et `bridge.py` conservent les points d’entrée de compatibilité.

## Connexion matérielle

Sur Windows, le mode réel utilise WSL, une interface CAN configurée et le SDK [AgileX Piper](https://github.com/agilexrobotics/piper_sdk). Le SDK n’est pas inclus. Adapter les paramètres `distro`, `channel` et `wsl_python` de `Piper(...)` ; le chemin générique fourni est `/opt/piperx/.venv/bin/python`. Le SDK 0.7.0a1 et le protocole V3 ont été utilisés lors de la préparation locale.

Ne connecter qu’un contrôleur au robot. Avant toute activation, vérifier le modèle, le firmware, les limites, la calibration, la zone de travail et l’accès à l’arrêt matériel. Le script `Preparer-CAN.ps1` est un utilitaire Windows/WSL à adapter à l’installation.

## Protections et limites

La connexion ne demande pas l’activation des moteurs. Les commandes vérifient notamment le consentement, les limites, les défauts, le mode du robot et la fraîcheur des retours. Des temporisations limitent l’autorisation et les commandes continues.

**L’arrêt logiciel et les watchdogs ne remplacent jamais l’arrêt d’urgence matériel.** Ils dépendent du processus, du SDK et de la liaison CAN. Les limites de consigne ne garantissent pas la vitesse physique. Aucun évitement de collision, asservissement cartésien, planificateur autonome ou garantie de temps réel dur n’est fourni. Une connexion en lecture ne désactive pas des moteurs déjà actifs.

## Validation

Les **22 tests automatisés passent sous Python 3.12**, en simulation ou avec SDK factice : consentement, expiration, limites et conversions, retours périmés, pince, renouvellement, cibles progressives, accélération et confirmation de fermeture. Aucun mouvement matériel n’est réalisé par cette suite. Ces tests ne constituent pas une validation de sécurité du robot réel.
