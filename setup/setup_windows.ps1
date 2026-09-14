<#
.SYNOPSIS
    Prepare Windows so the robot can actually reach the Docker container.

.DESCRIPTION
    Docker Desktop publishes ports on the PC, but Windows Firewall blocks the
    inbound traffic by default. The container starts, the web UI works from the
    same PC, and then the ESP32 cannot reach the micro-ROS agent and nothing
    connects - with no error to explain why, because the packets are dropped
    before Docker ever sees them.

    That is the "extra setup" this script removes. It:

      1. checks Docker Desktop is installed and running
      2. opens TCP 8080 (web UI) and UDP 8888 (micro-ROS agent) inbound
      3. prints the LAN address to put in the firmware
      4. checks the ports are actually listening once the stack is up

    Run it once, as Administrator:

        powershell -ExecutionPolicy Bypass -File setup\setup_windows.ps1

    Undo everything it did:

        powershell -ExecutionPolicy Bypass -File setup\setup_windows.ps1 -Remove
#>
[CmdletBinding()]
param(
    [switch]$Remove,
    [int]$WebPort   = 8080,
    [int]$AgentPort = 8888
)

$ErrorActionPreference = 'Stop'
$RuleWeb   = 'GPS_Localize web UI'
$RuleAgent = 'GPS_Localize micro-ROS agent'

function Say  ($m) { Write-Host "  $m" }
function Ok   ($m) { Write-Host "  OK    $m"   -ForegroundColor Green }
function Warn ($m) { Write-Host "  warn  $m"   -ForegroundColor Yellow }
function Bad  ($m) { Write-Host "  FAIL  $m"   -ForegroundColor Red }

function Test-Admin {
    $id = [Security.Principal.WindowsIdentity]::GetCurrent()
    (New-Object Security.Principal.WindowsPrincipal $id).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator)
}

Write-Host ""
Write-Host "GPS_Localize - Windows setup" -ForegroundColor Cyan
Write-Host ("=" * 60)

if (-not (Test-Admin)) {
    Bad "not running as Administrator"
    Say "Firewall rules need it. Right-click PowerShell -> Run as Administrator,"
    Say "then run this script again."
    exit 1
}

# ---------------------------------------------------------------- remove ----
if ($Remove) {
    Write-Host "`n[removing firewall rules]"
    foreach ($name in @($RuleWeb, $RuleAgent)) {
        if (Get-NetFirewallRule -DisplayName $name -ErrorAction SilentlyContinue) {
            Remove-NetFirewallRule -DisplayName $name
            Ok "removed: $name"
        } else {
            Say "not present: $name"
        }
    }
    Write-Host "`ndone`n"
    exit 0
}

# ---------------------------------------------------------------- docker ----
Write-Host "`n[docker]"
$docker = Get-Command docker -ErrorAction SilentlyContinue
if (-not $docker) {
    Bad "Docker not found"
    Say "Install Docker Desktop: https://www.docker.com/products/docker-desktop/"
    Say "Then run this script again."
    exit 1
}
Ok "docker found: $($docker.Source)"

docker info *> $null
if ($LASTEXITCODE -ne 0) {
    Warn "Docker is installed but not running - start Docker Desktop and wait for"
    Say  "the whale icon to stop animating, then run this script again."
} else {
    Ok "Docker Desktop is running"
}

# -------------------------------------------------------------- firewall ----
Write-Host "`n[firewall]"
Say "Docker publishes the ports, but Windows blocks inbound traffic to them by"
Say "default. Without these two rules the ESP32 cannot reach the agent and the"
Say "link silently never comes up."

$rules = @(
    @{ Name = $RuleWeb;   Port = $WebPort;   Proto = 'TCP'; What = 'web UI, so a phone can open it' },
    @{ Name = $RuleAgent; Port = $AgentPort; Proto = 'UDP'; What = 'micro-ROS agent, so the robot can connect' }
)
foreach ($r in $rules) {
    $existing = Get-NetFirewallRule -DisplayName $r.Name -ErrorAction SilentlyContinue
    if ($existing) {
        Ok "already allowed: $($r.Proto) $($r.Port)  ($($r.What))"
        continue
    }
    New-NetFirewallRule -DisplayName $r.Name `
        -Direction Inbound -Action Allow `
        -Protocol $r.Proto -LocalPort $r.Port `
        -Profile Private,Domain `
        -Description "GPS_Localize: $($r.What)" | Out-Null
    Ok "allowed: $($r.Proto) $($r.Port)  ($($r.What))"
}
Say ""
Say "Rules apply to Private and Domain networks only, not Public. If your Wi-Fi"
Say "is set to Public, Windows will still block it - change the network to"
Say "Private in Settings, or the robot will not connect."

$profiles = Get-NetConnectionProfile | Where-Object { $_.IPv4Connectivity -ne 'Disconnected' }
foreach ($p in $profiles) {
    if ($p.NetworkCategory -eq 'Public') {
        Warn "'$($p.Name)' is a Public network - the rules above will NOT apply to it"
        Say  "  Set-NetConnectionProfile -Name '$($p.Name)' -NetworkCategory Private"
    } else {
        Ok "'$($p.Name)' is $($p.NetworkCategory)"
    }
}

# --------------------------------------------------------------- address ----
Write-Host "`n[address to put in the firmware]"
$addrs = Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.IPAddress -notmatch '^(127\.|169\.254\.)' -and
                   $_.InterfaceAlias -notmatch 'Loopback|WSL|Hyper-V|vEthernet' } |
    Select-Object -ExpandProperty IPAddress -Unique

if (-not $addrs) {
    Warn "no LAN address found - is this PC on Wi-Fi or Ethernet?"
} else {
    foreach ($a in $addrs) { Ok "this PC: $a" }
    $first = @($addrs)[0]
    Say ""
    Say "Put that in firmware/config/network.h:"
    Say ""
    Say "    static const IPAddress AGENT_IP($($first -replace '\.', ', '));"
    Say ""
    Say "Use the PC address, NOT a 172.x docker address - Docker Desktop forwards"
    Say "the published port from the PC into the container."
}

# --------------------------------------------------------------- listening --
Write-Host "`n[are the ports open right now]"
foreach ($r in $rules) {
    $listening = if ($r.Proto -eq 'TCP') {
        Get-NetTCPConnection -LocalPort $r.Port -State Listen -ErrorAction SilentlyContinue
    } else {
        Get-NetUDPEndpoint -LocalPort $r.Port -ErrorAction SilentlyContinue
    }
    if ($listening) {
        Ok "$($r.Proto) $($r.Port) is listening"
    } else {
        Say "$($r.Proto) $($r.Port) not listening yet - normal if the stack is not started"
    }
}

Write-Host "`n$('=' * 60)"
Write-Host "next:" -ForegroundColor Cyan
Say "  docker compose -f docker-compose.yml -f docker-compose.windows.yml up -d"
Say "  python tools\test_windows.py"
Say ""
Say "then open http://localhost:8080"
Write-Host ""
