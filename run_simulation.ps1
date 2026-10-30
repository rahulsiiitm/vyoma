param(
    [string]$Container = 'vyoma-dev',
    [string]$Workspace,
    [string]$InstallPath = '/tmp/vyoma-runtime/install',
    [string]$BuildBase = '/tmp/vyoma-runtime/build',
    [string]$Px4Source,
    [switch]$Headless = $true,
    [switch]$Gui,
    [string]$WslDistribution = 'Ubuntu',
    [switch]$Training,
    [string]$ModelPath,
    [double]$DepthRate = 0,
    [double]$Duration = 0,
    [switch]$SkipBuild
)

$ErrorActionPreference = 'Stop'
if ([double]::IsNaN($Duration) -or [double]::IsInfinity($Duration) -or $Duration -lt 0) {
    throw 'Duration must be finite and nonnegative.'
}
if ([double]::IsNaN($DepthRate) -or [double]::IsInfinity($DepthRate) -or $DepthRate -lt 0) {
    throw 'DepthRate must be finite and nonnegative.'
}
$inspection = docker inspect $Container | ConvertFrom-Json
if ($LASTEXITCODE -ne 0) { throw "Container '$Container' does not exist. Create the ROS development container first." }
if (-not $Workspace) {
    $repository = (Resolve-Path -LiteralPath $PSScriptRoot).Path
    $mount = $inspection[0].Mounts | Where-Object { $_.Source -ieq $repository } | Select-Object -First 1
    if (-not $mount) { throw 'Cannot find this repository in the container mounts. Supply -Workspace explicitly.' }
    $Workspace = $mount.Destination
}
if (-not $inspection[0].State.Running) {
    docker start $Container
    if ($LASTEXITCODE -ne 0) { throw 'Could not start the simulation container.' }
}
if (-not $SkipBuild) {
    $buildCommand = @'
        set -e
        source /opt/ros/humble/setup.bash
        cd "$1"
        colcon --log-base /tmp/vyoma-runtime/log build --build-base "$2" --install-base "$3" \
          --symlink-install --packages-select px4_msgs semantic_nav_pkg semantic_vision_pkg rl_navigation_pkg
'@
    docker exec $Container bash -lc ($buildCommand -replace "`r`n", "`n") vyoma-build $Workspace $BuildBase $InstallPath
    if ($LASTEXITCODE -ne 0) { throw 'ROS workspace build failed.' }
}
if ($Gui) {
    wsl -d $WslDistribution -- test -S /tmp/.X11-unix/X0
    if ($LASTEXITCODE -ne 0) { throw 'WSLg display is unavailable in the selected WSL distribution.' }
    $displayAddress = ((wsl -d $WslDistribution -- hostname -I).Trim() -split '\s+')[0]
    # WSL treats backslashes as shell escapes in native command arguments.
    $windowsRelayPath = (Join-Path $PSScriptRoot 'scripts/wsl_x11_bridge.py').Replace('\', '/')
    $relayPathResult = wsl -d $WslDistribution -- wslpath -u $windowsRelayPath
    if ($LASTEXITCODE -ne 0 -or -not $relayPathResult) { throw 'Cannot resolve the WSL display relay path.' }
    $relayPath = $relayPathResult.Trim()
    Start-Process -FilePath wsl.exe -WindowStyle Hidden -ArgumentList @(
        '-d', $WslDistribution, '--', 'python3', "`"$relayPath`"", '--listen', $displayAddress)
    $displayReady = @'
import socket, sys, time
for attempt in range(50):
    try:
        socket.create_connection((sys.argv[1], 6000), timeout=1).close()
        break
    except OSError:
        time.sleep(0.1)
else:
    raise SystemExit('Cannot reach the WSLg display relay from the container')
'@
    docker exec $Container python3 -c $displayReady $displayAddress
    if ($LASTEXITCODE -ne 0) { throw 'Cannot connect the simulator display.' }
    docker exec $Container pgrep -f '^gz sim -g' | Out-Null
    if ($LASTEXITCODE -ne 0) {
        docker exec -d -e "DISPLAY=${displayAddress}:0" -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 `
            $Container bash -lc 'exec gz sim -g > /tmp/vyoma-gui.log 2>&1'
        if ($LASTEXITCODE -ne 0) { throw 'Could not launch the Gazebo viewer.' }
    }
}
$sessionArguments = @('--workspace', $Workspace)
if ($Headless) { $sessionArguments += '--headless' }
if ($Training) { $sessionArguments += '--training' }
if ($ModelPath) { $sessionArguments += @('--model-path', $ModelPath) }
if ($DepthRate -gt 0) { $sessionArguments += @('--depth-rate', $DepthRate.ToString([Globalization.CultureInfo]::InvariantCulture)) }
if ($Duration -gt 0) { $sessionArguments += @('--duration', $Duration.ToString([Globalization.CultureInfo]::InvariantCulture)) }
if ($Px4Source) { $sessionArguments += @('--px4-source', $Px4Source) }
$sessionCommand = @'
    set -e
    source /opt/ros/humble/setup.bash
    source "$1/setup.bash"
    script=$2
    shift 2
    exec python3 "$script" "$@"
'@
docker exec -i $Container bash -lc ($sessionCommand -replace "`r`n", "`n") vyoma-session $InstallPath "$Workspace/scripts/simulation_session.py" @sessionArguments
if ($LASTEXITCODE -ne 0) { throw 'Simulation session failed. Check artifacts/simulation logs.' }
