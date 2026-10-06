param(
    [Parameter(Mandatory = $true)]
    [string]$BiosPath,

    [string]$BuildDir = "build-ps2",
    [int]$Runs = 3,
    [UInt64]$Budget = 400000000,
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $BiosPath)) {
    throw "BIOS not found: $BiosPath"
}
if ($Runs -lt 1) {
    throw "Runs must be at least 1."
}

$trace = Join-Path $BuildDir "Release\vibestation_ps2_bios_trace.exe"
if (-not $SkipBuild) {
    cmake --build $BuildDir --config Release --target vibestation_ps2_bios_trace
    if ($LASTEXITCODE -ne 0) {
        throw "Release trace build failed."
    }
}
if (-not (Test-Path -LiteralPath $trace)) {
    $trace = Join-Path $BuildDir "vibestation_ps2_bios_trace"
}
if (-not (Test-Path -LiteralPath $trace)) {
    throw "Trace executable not found under $BuildDir."
}

function Parse-Metric {
    param([string]$Text, [string]$Name)
    $match = [regex]::Match(
        $Text,
        "(?m)(?:^|\s)" + [regex]::Escape($Name) + "=([^\s]+)")
    if (-not $match.Success) { return $null }
    return $match.Groups[1].Value
}

function Metric-OrZero {
    param([string]$Text, [string]$Name)
    $value = Parse-Metric $Text $Name
    if ($null -eq $value -or $value -eq "") {
        return "0"
    }
    return $value
}

function Run-Backend {
    param([string]$Name, [string[]]$ExtraArgs)

    $rows = @()
    for ($i = 1; $i -le $Runs; ++$i) {
        Write-Host "[$Name] run $i/$Runs"
        $arguments = @(
            $BiosPath,
            [string]$Budget,
            "--profile",
            "--gs-thread"
        ) + $ExtraArgs

        $output = & $trace @arguments 2>&1 | Out-String
        if ($LASTEXITCODE -ne 0) {
            Write-Host $output
            throw "$Name trace failed with exit code $LASTEXITCODE."
        }

        $row = [pscustomobject]@{
            Backend = $Name
            Run = $i
            FieldRate = [double](Parse-Metric $output "PROFILE_FIELD_RATE")
            RunMs = [double](Parse-Metric $output "PROFILE_RUN_MS")
            DisplayHash = Parse-Metric $output "DISPLAY_HASH"
            RasterPixels = [UInt64](Parse-Metric $output "GS_RASTER_PIXELS")
            DynarecInstructions = [UInt64](Metric-OrZero $output "EE_DYNAREC_INSTRUCTIONS")
            DynarecBlocks = [UInt64](Metric-OrZero $output "EE_DYNAREC_BLOCKS_EXECUTED")
            DynarecCompiled = [UInt64](Metric-OrZero $output "EE_DYNAREC_BLOCKS_COMPILED")
            DispatchCalls = [UInt64](Metric-OrZero $output "EE_DYNAREC_DISPATCH_CALLS")
            LinkHits = [UInt64](Metric-OrZero $output "EE_DYNAREC_LINK_HITS")
            LinkMisses = [UInt64](Metric-OrZero $output "EE_DYNAREC_LINK_MISSES")
            GuardExits = [UInt64](Metric-OrZero $output "EE_DYNAREC_GUARD_EXITS")
            DeadlineExits = [UInt64](Metric-OrZero $output "EE_DYNAREC_DEADLINE_EXITS")
            UnsupportedExits = [UInt64](Metric-OrZero $output "EE_DYNAREC_UNSUPPORTED_EXITS")
            Cop0Exits = [UInt64](Metric-OrZero $output "EE_DYNAREC_COP0_WRITE_EXITS")
            FastmemLoads = [UInt64](Metric-OrZero $output "EE_DYNAREC_FASTMEM_LOADS")
            FastmemStores = [UInt64](Metric-OrZero $output "EE_DYNAREC_FASTMEM_STORES")
            RegisterCacheHits = [UInt64](Metric-OrZero $output "EE_DYNAREC_REGCACHE_HITS")
            CacheFlushes = [UInt64](Metric-OrZero $output "EE_DYNAREC_CACHE_FLUSHES")
            UnsupportedTop = [regex]::Match($output, "(?m)^EE_DYNAREC_UNSUPPORTED_TOP.*$").Value
        }
        $rows += $row
        $row | Format-List
    }
    return ,$rows
}

function Median {
    param([double[]]$Values)
    $sorted = @($Values | Sort-Object)
    $count = $sorted.Count
    if (($count % 2) -eq 1) {
        return $sorted[[int]($count / 2)]
    }
    return ($sorted[$count / 2 - 1] + $sorted[$count / 2]) / 2.0
}

$interpreter = Run-Backend "cached-interpreter" @()
$dynarec = Run-Backend "second-gen-x64-dynarec" @("--ee-dynarec")

$referenceHash = $interpreter[0].DisplayHash
$referencePixels = $interpreter[0].RasterPixels
foreach ($row in $interpreter + $dynarec) {
    if ($row.DisplayHash -ne $referenceHash) {
        throw "Display hash mismatch: $($row.Backend) run $($row.Run) produced $($row.DisplayHash), expected $referenceHash."
    }
    if ($row.RasterPixels -ne $referencePixels) {
        throw "Raster pixel mismatch: $($row.Backend) run $($row.Run) produced $($row.RasterPixels), expected $referencePixels."
    }
}

$summary = @(
    [pscustomobject]@{
        Backend = "cached-interpreter"
        MedianFieldRate = Median @($interpreter.FieldRate)
        MedianRunMs = Median @($interpreter.RunMs)
        DisplayHash = $referenceHash
        RasterPixels = $referencePixels
    },
    [pscustomobject]@{
        Backend = "second-gen-x64-dynarec"
        MedianFieldRate = Median @($dynarec.FieldRate)
        MedianRunMs = Median @($dynarec.RunMs)
        DisplayHash = $referenceHash
        RasterPixels = $referencePixels
    }
)

Write-Host ""
Write-Host "400M EE backend comparison"
$summary | Format-Table -AutoSize

$dynMedian = Median @($dynarec.RunMs)
$intMedian = Median @($interpreter.RunMs)
$delta = (($intMedian - $dynMedian) / $intMedian) * 100.0
Write-Host ("Dynarec wall-time delta vs interpreter: {0:N2}%" -f $delta)
Write-Host ("Reference output: DISPLAY_HASH={0} GS_RASTER_PIXELS={1}" -f $referenceHash, $referencePixels)

$lastDynarec = $dynarec[$dynarec.Count - 1]
$avgBlock = if ($lastDynarec.DynarecBlocks -ne 0) { $lastDynarec.DynarecInstructions / [double]$lastDynarec.DynarecBlocks } else { 0.0 }
$avgDispatch = if ($lastDynarec.DispatchCalls -ne 0) { $lastDynarec.DynarecInstructions / [double]$lastDynarec.DispatchCalls } else { 0.0 }
$linkTotal = $lastDynarec.LinkHits + $lastDynarec.LinkMisses
$linkRate = if ($linkTotal -ne 0) { 100.0 * $lastDynarec.LinkHits / [double]$linkTotal } else { 0.0 }

Write-Host ""
Write-Host ("Second-gen coverage: {0} native instructions, {1} blocks, {2:N2} instructions/block" -f $lastDynarec.DynarecInstructions, $lastDynarec.DynarecBlocks, $avgBlock)
Write-Host ("Dispatch efficiency: {0} calls, {1:N2} native instructions/dispatch" -f $lastDynarec.DispatchCalls, $avgDispatch)
Write-Host ("Successor links: {0} hits / {1} misses ({2:N1}% hit rate)" -f $lastDynarec.LinkHits, $lastDynarec.LinkMisses, $linkRate)
Write-Host ("Exits: guard={0} deadline={1} unsupported={2} cop0={3}" -f $lastDynarec.GuardExits, $lastDynarec.DeadlineExits, $lastDynarec.UnsupportedExits, $lastDynarec.Cop0Exits)
Write-Host ("Fastmem: loads={0} stores={1}; reg-cache hits={2}; cache flushes={3}" -f $lastDynarec.FastmemLoads, $lastDynarec.FastmemStores, $lastDynarec.RegisterCacheHits, $lastDynarec.CacheFlushes)
if ($lastDynarec.UnsupportedTop -ne "") { Write-Host $lastDynarec.UnsupportedTop }
