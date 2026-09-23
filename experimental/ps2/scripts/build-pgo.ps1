param(
    [Parameter(Mandatory = $true)]
    [string]$BiosPath,

    [string]$Generator = 'Visual Studio 18 2026'
)

$ErrorActionPreference = 'Stop'
$biosFile = Get-Item -LiteralPath $BiosPath
if (-not $biosFile.PSIsContainer -and $biosFile.Length -eq 4MB) {
    $resolvedBios = $biosFile.FullName
} else {
    throw 'A 4 MiB ROM0 BIOS file is required.'
}

$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..\..')).Path
$ps2Source = Join-Path $repoRoot 'experimental\ps2'
$buildRoot = Join-Path $repoRoot 'build-ps2-pgo'
$releaseRoot = Join-Path $buildRoot 'Release'
$trace = Join-Path $releaseRoot 'vibestation_ps2_bios_trace.exe'
$profile = Join-Path $releaseRoot 'vibestation_ps2_bios_trace.pgd'

function Assert-Success([string]$Stage) {
    if ($LASTEXITCODE -ne 0) {
        throw "$Stage failed (exit code $LASTEXITCODE)."
    }
}

# The instrumented program must run once to train MSVC's link-time optimizer.
& cmake -S $ps2Source -B $buildRoot -G $Generator -A x64 `
    '-DVIBESTATION_PS2_ENABLE_UI=OFF' `
    '-DCMAKE_EXE_LINKER_FLAGS_RELEASE=/GENPROFILE'
Assert-Success 'PGO instrumentation configuration'
& cmake --build $buildRoot --config Release --target vibestation_ps2_bios_trace -j 2
Assert-Success 'PGO instrumentation build'

# pgort140.dll is a build-time training dependency, not a shipped dependency
# of the optimized executable. Locate the matching installed VS x64 runtime.
$vswhere = Join-Path ${env:ProgramFiles(x86)} `
    'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere)) {
    throw 'Visual Studio vswhere.exe was not found.'
}
$vsRoot = & $vswhere -latest -products '*' -property installationPath
if (-not $vsRoot) {
    throw 'An installed Visual Studio with MSVC PGO tools was not found.'
}
$toolVersions = Get-ChildItem -LiteralPath (Join-Path $vsRoot 'VC\Tools\MSVC') `
    -Directory | Sort-Object Name -Descending
$pgoRuntime = $null
foreach ($toolVersion in $toolVersions) {
    $candidate = Join-Path $toolVersion.FullName 'bin\Hostx64\x64\pgort140.dll'
    if (Test-Path -LiteralPath $candidate) {
        $pgoRuntime = $candidate
        break
    }
}
if (-not $pgoRuntime) {
    throw 'The x64 MSVC PGO runtime pgort140.dll was not found.'
}
Copy-Item -LiteralPath $pgoRuntime -Destination (Join-Path $releaseRoot 'pgort140.dll') -Force

$trainingOutput = & $trace $resolvedBios 212000000 2>&1
$trainingExitCode = $LASTEXITCODE
if ($trainingExitCode -ne 0 -or
    -not ($trainingOutput -match 'TRACE_FIRST_VISIBLE')) {
    $trainingOutput | Select-Object -Last 40 | Write-Output
    throw 'PGO training did not reach a visible BIOS frame.'
}
$trainingOutput | Where-Object {
    $_ -match '^(TRACE_FIRST_VISIBLE|VISIBLE_FRAME_READY)'
} | Write-Output
if (-not (Test-Path -LiteralPath $profile)) {
    throw 'MSVC did not create the PGO profile database.'
}

# The trace profile covers the shared PS2 core. MSVC also applies it when
# linking the graphical app; UI-only functions receive normal optimization.
$profilePathForLinker = $profile.Replace('\', '/')
$profileFlags = "/USEPROFILE:PGD=$profilePathForLinker"
& cmake -S $ps2Source -B $buildRoot -G $Generator -A x64 `
    '-DVIBESTATION_PS2_ENABLE_UI=ON' `
    "-DCMAKE_EXE_LINKER_FLAGS_RELEASE=$profileFlags"
Assert-Success 'PGO optimized configuration'
& cmake --build $buildRoot --config Release `
    --target vibestation_ps2_bios_trace VibeStationPS2Lab -j 2
Assert-Success 'PGO optimized build'

Write-Output "Optimized app: $(Join-Path $releaseRoot 'VibeStationPS2Lab.exe')"
Write-Output "Optimized trace: $trace"
