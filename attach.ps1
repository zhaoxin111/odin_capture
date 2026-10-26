$ErrorActionPreference = 'Stop'
$usbipd = 'C:\Program Files\usbipd-win\usbipd.exe'
if (!(Test-Path $usbipd)) { throw 'Install usbipd-win first.' }
$devices = (& $usbipd list) | Where-Object { $_ -match '2207:0019' }
if (@($devices).Count -ne 1) { & $usbipd list; throw 'Expected exactly one ODIN1 (2207:0019). Check the device list.' }
$busid = (($devices.Trim()) -split '\s+')[0]
if ($devices -match 'Not shared') {
  # usbipd bind requires an elevated PowerShell; attach does not.
  & $usbipd bind --busid $busid
  if ($LASTEXITCODE -ne 0) { throw 'Bind failed. Run this script in an administrator PowerShell.' }
}
if ($devices -notmatch 'Attached') {
  & $usbipd attach --wsl --busid $busid
  if ($LASTEXITCODE -ne 0) { throw 'Attach failed. Keep WSL open and check usbipd output.' }
}
& $usbipd list
