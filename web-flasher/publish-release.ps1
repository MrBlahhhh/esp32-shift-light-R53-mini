# Stage a shift light build for the web flasher: build both envs from the
# committed source, check the images, copy them into the Pages site and add
# the build to releases.json.
#
# What it writes into -Site (the gh-pages worktree, see README.md):
#
#   index.html, flasher.js, style.css      the page, copied from here
#   releases.json                          every published build, newest first
#   firmware/<build>/<env>/...             the images for that build
#
# The page links each build to the commit it came from, so unless -Bench is
# given this refuses to publish with uncommitted changes in anything that goes
# into the firmware, and it runs the build itself rather than trusting
# whatever is lying in .pio.
#
#   .\publish-release.ps1 -Notes "Silent app handshake"      next build number, both boards
#   .\publish-release.ps1 -Build 3 -Notes "..."              replace build 3
#   .\publish-release.ps1 -Bench -Site C:\tmp\flasher        stage a bench test from the working tree
#   .\publish-release.ps1 -Envs esp32-c3                     only this board

param(
    [string]$Site = "C:\Projects\esp32-shift-light-R53-mini-pages",
    [string[]]$Envs = @("esp32-c3", "esp32-s3-zero"),
    [int]$Build = 0,
    [string]$Notes = "",
    [switch]$Bench
)

$ErrorActionPreference = "Stop"
$Repo = Split-Path $PSScriptRoot -Parent
$SourceRepoUrl = "https://github.com/MrBlahhhh/esp32-shift-light-R53-mini"

# Image header byte 12 is the chip id the image was built for. The page picks
# the folder by chip; this makes sure the folder holds that chip's image.
$ChipIds = @{ "esp32-c3" = 5; "esp32-s3-zero" = 9 }

# Run git and return its stdout. Continue, not Stop: Windows PowerShell turns
# any stderr line from a native program into a terminating error.
function Run-Git {
    $ErrorActionPreference = "Continue"
    $out = & git.exe -C $Repo @args 2>$null
    return @{ Out = $out; Ok = ($LASTEXITCODE -eq 0) }
}

function Hex([uint32]$n) { return "0x" + $n.ToString("x") }

function Hash([string]$path, [string]$algorithm) {
    return (Get-FileHash -Algorithm $algorithm -LiteralPath $path).Hash.ToLower()
}

# partitions.bin is 32-byte records: AA 50, type, subtype, offset, size,
# 16-byte label, flags. It ends at an EB EB md5 record or at erased flash.
function Read-PartitionTable([string]$path) {
    $bytes = [System.IO.File]::ReadAllBytes($path)
    $parts = [ordered]@{}
    for ($i = 0; $i + 32 -le $bytes.Length; $i += 32) {
        if ($bytes[$i] -ne 0xAA -or $bytes[$i + 1] -ne 0x50) { break }
        $label = [System.Text.Encoding]::ASCII.GetString($bytes, $i + 12, 16).TrimEnd([char]0)
        $parts[$label] = [pscustomobject]@{
            Offset = [BitConverter]::ToUInt32($bytes, $i + 4)
            Size = [BitConverter]::ToUInt32($bytes, $i + 8)
        }
    }
    return $parts
}

function Check-Image([string]$path, [int]$chipId) {
    $bytes = [System.IO.File]::ReadAllBytes($path)
    if ($bytes[0] -ne 0xE9) { throw "$path is not an ESP image (first byte $(Hex $bytes[0]))" }
    if ($bytes[12] -ne $chipId) { throw "$path is built for chip id $($bytes[12]), expected $chipId" }
}

# ---------------------------------------------------------------- source

$sourceCommit = $null
$dirty = Run-Git status --porcelain -- src include lib platformio.ini
if ($dirty.Out) {
    Write-Host "Uncommitted changes that go into the firmware:"
    $dirty.Out | ForEach-Object { Write-Host "  $_" }
    if (-not $Bench) { throw "commit them first, or use -Bench for a local test" }
}
if (-not $Bench) {
    $sourceCommit = "$((Run-Git rev-parse HEAD).Out)".Trim()
    $pushed = Run-Git branch -r --contains $sourceCommit
    if (-not $pushed.Out) { Write-Host "WARNING: $($sourceCommit.Substring(0, 7)) is not on any remote branch yet. Push it before the page goes live, or its source link is a 404." }
}

# ---------------------------------------------------------------- build

$pio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"
if (-not (Test-Path $pio)) { $pio = "pio" }
$pioArgs = @("run", "-d", $Repo)
foreach ($envName in $Envs) { $pioArgs += @("-e", $envName) }
Write-Host "Building $($Envs -join ', ')"
$ErrorActionPreference = "Continue"
& $pio @pioArgs
$built = $LASTEXITCODE -eq 0
$ErrorActionPreference = "Stop"
if (-not $built) { throw "pio run failed" }

# The same file `pio run -t upload` writes at 0xe000: otadata with ota_seq 1,
# which boots app0.
$bootApp0 = Join-Path $env:USERPROFILE ".platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin"
$bootApp0Bytes = [System.IO.File]::ReadAllBytes($bootApp0)
if ($bootApp0Bytes.Length -ne 0x2000 -or [BitConverter]::ToUInt32($bootApp0Bytes, 0) -ne 1) {
    throw "$bootApp0 is not the 8 KB otadata with ota_seq 1 this script expects"
}

# ---------------------------------------------------------------- build number

$manifestPath = Join-Path $Site "releases.json"
$others = @()
if (Test-Path $manifestPath) {
    $others = @((Get-Content $manifestPath -Raw | ConvertFrom-Json).releases)
}
if ($Build -le 0) {
    $Build = 1 + (@($others | ForEach-Object { [int]$_.build }) + 0 | Measure-Object -Maximum).Maximum
}
$others = @($others | Where-Object { [int]$_.build -ne $Build })
Write-Host "Build $Build$(if ($sourceCommit) { " from $($sourceCommit.Substring(0, 7))" } else { ' (bench)' })"

# ---------------------------------------------------------------- stage each board

$boardEntries = [ordered]@{}

foreach ($envName in $Envs) {
    Write-Host "== $envName"
    if (-not $ChipIds.ContainsKey($envName)) { throw "no chip id for $envName; add it to `$ChipIds" }
    $dir = Join-Path $Repo ".pio\build\$envName"
    $bootloader = Join-Path $dir "bootloader.bin"
    $table = Join-Path $dir "partitions.bin"
    $app = Join-Path $dir "firmware.bin"
    foreach ($path in @($bootloader, $table, $app)) {
        if (-not (Test-Path $path)) { throw "$path is missing; did the build run?" }
    }
    Check-Image $bootloader $ChipIds[$envName]
    Check-Image $app $ChipIds[$envName]

    $parts = Read-PartitionTable $table
    foreach ($name in @("nvs", "otadata", "app0")) {
        if (-not $parts.Contains($name)) { throw "$envName partition table has no $name" }
    }
    # The page writes an update at these two addresses and never the rest; a
    # table that moved them is a different layout and needs a fresh install.
    if ($parts["app0"].Offset -ne 0x10000 -or $parts["otadata"].Offset -ne 0xe000) {
        throw "$envName has app0 at $(Hex $parts['app0'].Offset) and otadata at $(Hex $parts['otadata'].Offset), expected 0x10000 and 0xe000"
    }
    $appSize = (Get-Item $app).Length
    if ($appSize -gt $parts["app0"].Size) { throw "$envName firmware.bin is $appSize bytes, app0 holds $($parts['app0'].Size)" }

    $outDir = Join-Path $Site "firmware\$Build\$envName"
    if (Test-Path $outDir) { Remove-Item -Recurse -Force $outDir }
    New-Item -ItemType Directory -Force $outDir | Out-Null
    # The app gets the build in its name so a downloaded copy says what it is.
    $appName = "shiftlight-$envName-build$Build.bin"
    Copy-Item $bootloader -Destination (Join-Path $outDir "bootloader.bin")
    Copy-Item $table -Destination (Join-Path $outDir "partitions.bin")
    Copy-Item $bootApp0 -Destination (Join-Path $outDir "boot_app0.bin")
    Copy-Item $app -Destination (Join-Path $outDir $appName)

    function File-Entry([string]$name, [uint32]$offset, [string]$what) {
        $path = Join-Path $outDir $name
        return [ordered]@{
            what = $what
            path = "firmware/$Build/$envName/$name"
            offset = Hex $offset
            size = (Get-Item $path).Length
            md5 = Hash $path MD5
            sha256 = Hash $path SHA256
        }
    }

    # Update writes otadata as well as the app. This firmware has no OTA, but
    # a board that came with other firmware on the same min_spiffs layout may
    # have been left booting app1, and writing app0 alone would change nothing
    # it runs. Otadata goes after the app: if the write stops in between,
    # otadata still names whatever the board booted before. NVS at 0x9000,
    # where the saved config lives, is never written.
    $update = @(
        (File-Entry $appName 0x10000 "app"),
        (File-Entry "boot_app0.bin" 0xe000 "otadata")
    )
    # The C3 and S3 load the second-stage bootloader from 0x0 (the original
    # ESP32 used 0x1000; this firmware doesn't build for it).
    $fresh = @(
        (File-Entry "bootloader.bin" 0x0 "bootloader"),
        (File-Entry "partitions.bin" 0x8000 "partitions"),
        (File-Entry "boot_app0.bin" 0xe000 "otadata"),
        (File-Entry $appName 0x10000 "app")
    )

    $partMap = [ordered]@{}
    foreach ($name in $parts.Keys) { $partMap[$name] = Hex $parts[$name].Offset }

    $boardEntries[$envName] = [ordered]@{
        partitionTable = [ordered]@{ offset = "0x8000"; size = (Get-Item $table).Length; md5 = Hash $table MD5 }
        partitions = $partMap
        update = $update
        fresh = $fresh
    }
    Write-Host "  app $appSize bytes of $($parts['app0'].Size)"
    Write-Host "  update: $(($update | ForEach-Object { "$($_.offset) $($_.what)" }) -join ', ')"
    Write-Host "  fresh:  erase, $(($fresh | ForEach-Object { "$($_.offset) $($_.what)" }) -join ', ')"
}

# ---------------------------------------------------------------- releases.json

$release = [ordered]@{
    build = $Build
    date = (Get-Date).ToString("yyyy-MM-dd")
    bench = [bool]$Bench
    notes = $Notes
    sourceCommit = $sourceCommit
    boards = $boardEntries
}

$releases = @(@($release) + $others | Sort-Object -Property { [int]$_.build } -Descending)
$manifest = [ordered]@{ schema = 1; sourceRepo = $SourceRepoUrl; releases = $releases }
$json = ConvertTo-Json -InputObject $manifest -Depth 12
[System.IO.File]::WriteAllText($manifestPath, $json, (New-Object System.Text.UTF8Encoding $false))

foreach ($page in @("index.html", "flasher.js", "style.css")) {
    Copy-Item (Join-Path $PSScriptRoot $page) -Destination $Site -Force
}
# Serve the files as they are; Pages would otherwise run them through Jekyll.
$noJekyll = Join-Path $Site ".nojekyll"
if (-not (Test-Path $noJekyll)) { New-Item -ItemType File $noJekyll | Out-Null }

Write-Host ""
Write-Host "Staged build $Build in $Site."
Write-Host "Try it locally:  cd $Site; python -m http.server 8000   then open http://localhost:8000 in Chrome or Edge"
if (-not $Bench) {
    Write-Host "Then commit and push the site (see README.md)."
}
