param([Parameter(Mandatory=$true)][string]$ReadyPath)
$ErrorActionPreference = 'Stop'
$ready = Get-Content -Raw $ReadyPath | ConvertFrom-Json
$editor = Get-Process -Id ([int]$ready.pid) -ErrorAction Stop
if ($editor.ProcessName -ne 'UnrealEditor' -or $editor.MainWindowHandle -eq [IntPtr]::Zero) {
    throw 'AirGround editor window was not found'
}
Add-Type @'
using System;
using System.Runtime.InteropServices;
public class AirGroundCaptureWindow {
    [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr hWnd);
    [DllImport("user32.dll")] public static extern bool ShowWindowAsync(IntPtr hWnd, int command);
}
'@
$wasMinimized = [AirGroundCaptureWindow]::IsIconic($editor.MainWindowHandle)
if ($wasMinimized) {
    [void][AirGroundCaptureWindow]::ShowWindowAsync($editor.MainWindowHandle, 9)
    Start-Sleep -Milliseconds 500
}
$stillMinimized = [AirGroundCaptureWindow]::IsIconic($editor.MainWindowHandle)
if ($stillMinimized) { throw 'Restore the UE editor window before starting camera capture' }
[PSCustomObject]@{pid=$editor.Id; was_minimized=$wasMinimized; minimized=$stillMinimized} | ConvertTo-Json -Compress
