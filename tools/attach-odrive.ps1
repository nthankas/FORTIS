#Requires -Version 5.1
<#
.SYNOPSIS
    Forward any ODrive USB device from Windows into WSL2 so the fortis-dev
    container can reach it as /dev/ttyACM*.

.DESCRIPTION
    Idempotent: safe to re-run after a replug or Windows restart. The
    `usbipd bind` step requires admin and only needs to happen once per
    physical device. The `usbipd attach` step does NOT require admin and
    must be re-run after every unplug or Windows reboot.

    Pairs with tools/check-odrive-usb.sh (run inside WSL or fortis-dev).
    Full setup is documented in tools/odrive_calibrate.md.

.NOTES
    If PowerShell refuses to run the script ("running scripts is disabled
    on this system"), invoke it like this instead:
        powershell -ExecutionPolicy Bypass -File .\tools\attach-odrive.ps1
#>

$ErrorActionPreference = 'Stop'

if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
    Write-Host "usbipd-win not found. Install with:" -ForegroundColor Red
    Write-Host "  winget install --interactive --exact dorssel.usbipd-win"
    exit 1
}

$rawList = & usbipd list 2>&1
$odriveLines = $rawList | Where-Object { $_ -match 'ODrive' }

if (-not $odriveLines) {
    Write-Host "No ODrive devices found in 'usbipd list'." -ForegroundColor Yellow
    Write-Host "Confirm the S1 is plugged in (USB-C to host) and powered."
    Write-Host ""
    Write-Host "Current usbipd list output:"
    Write-Host ($rawList -join "`n")
    exit 1
}

foreach ($line in $odriveLines) {
    # usbipd list pads columns with 2+ spaces; split on that to keep
    # multi-word descriptions intact.
    $tokens = $line -split '\s{2,}'
    if ($tokens.Count -lt 4) {
        Write-Host "  (could not parse line: $line)" -ForegroundColor Yellow
        continue
    }
    $busid = $tokens[0].Trim()
    $state = $tokens[-1].Trim()
    $desc  = ($tokens[2..($tokens.Count - 2)] -join ' ').Trim()

    Write-Host "ODrive at $busid : $desc [$state]"

    if ($state -eq 'Not shared') {
        Write-Host "  -> binding (admin required)..."
        & usbipd bind --busid $busid
    }

    if ($state -ne 'Attached') {
        Write-Host "  -> attaching to WSL..."
        & usbipd attach --wsl --busid $busid
    } else {
        Write-Host "  -> already attached"
    }
}

Write-Host ""
Write-Host "Inside WSL/container, verify with: ls /dev/ttyACM*" -ForegroundColor Green
