param(
    [Parameter(Mandatory=$true)][string]$EngineDir,
    [string]$PluginDir = (Join-Path $PSScriptRoot '../ue_plugin/SarCameraStream'),
    [string]$OutputDir = (Join-Path $env:TEMP 'SarCameraStreamBuild'),
    [string]$AutoSDKRoot = ''
)
$ErrorActionPreference = 'Stop'
$EngineDir = (Resolve-Path $EngineDir).ProviderPath
$PluginDir = (Resolve-Path $PluginDir).ProviderPath
if ($AutoSDKRoot) { $env:UE_SDKS_ROOT = (Resolve-Path $AutoSDKRoot).ProviderPath }
$uat = Join-Path $EngineDir 'Engine/Build/BatchFiles/RunUAT.bat'
& $uat BuildPlugin "-Plugin=$PluginDir/SarCameraStream.uplugin" "-Package=$OutputDir" -NoTargetPlatforms
if ($LASTEXITCODE -ne 0) { throw "UE plugin build failed ($LASTEXITCODE)" }
$binaryDir = Join-Path $PluginDir 'Binaries/Win64'
New-Item -ItemType Directory -Force $binaryDir | Out-Null
$files = @('Binaries/Win64/UnrealEditor-SarCameraStream.dll', 'Binaries/Win64/UnrealEditor.modules')
$hashes = [ordered]@{}
foreach ($file in $files) {
    Copy-Item (Join-Path $OutputDir $file) (Join-Path $PluginDir $file) -Force
    $hashes[$file] = (Get-FileHash (Join-Path $PluginDir $file) -Algorithm SHA256).Hash.ToLowerInvariant()
}
$modules = Get-Content (Join-Path $binaryDir 'UnrealEditor.modules') -Raw | ConvertFrom-Json
$version = Get-Content (Join-Path $EngineDir 'Engine/Build/Build.version') -Raw | ConvertFrom-Json
$manifest = [ordered]@{ engine="UE$($version.MajorVersion).$($version.MinorVersion).$($version.PatchVersion)"; build_id=$modules.BuildId; files=$hashes }
$manifest | ConvertTo-Json -Depth 4 | Set-Content (Join-Path $PluginDir 'manifest.json') -Encoding UTF8
Write-Host 'Native editor plugin and integrity manifest updated.'
