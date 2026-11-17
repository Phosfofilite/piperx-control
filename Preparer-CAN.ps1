$ErrorActionPreference = 'Stop'
Write-Host 'Preparation USB-CAN uniquement : aucune commande moteur.'
$taskUsb = 'C:\Program Files\usbipd-win\usbipd.exe'
& $taskUsb list
if ($LASTEXITCODE -ne 0) { throw 'usbipd indisponible' }
$taskBus = Read-Host 'BUSID du candleLight USB to CAN (exemple 2-2, verifier la liste)'
if ($taskBus -notmatch '^\d+-\d+(\.\d+)*$') { throw 'BUSID invalide' }
& $taskUsb attach --wsl Ubuntu-24.04 --busid $taskBus
if ($LASTEXITCODE -ne 0) { throw 'Rattachement USB impossible ; verifier que le peripherique est partage.' }
wsl -d Ubuntu-24.04 -u root -- ip link set can0 up type can bitrate 1000000
if ($LASTEXITCODE -ne 0) { throw 'Configuration de can0 impossible.' }
wsl -d Ubuntu-24.04 -- ip -details link show can0
Write-Host 'CAN pret. Lancer Lancer-PiperX.cmd puis choisir Bras reel / WSL.'
