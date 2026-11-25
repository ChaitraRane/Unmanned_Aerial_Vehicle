# Practical 4 launcher (Windows PowerShell).  Uses the UAV project's micromamba env "uavros" (pybullet, pymavlink, onnxruntime, opencv).
#   .\run.ps1                         # normal mission
#   .\run.ps1 -Scenario fault_ugv_link
#   .\run.ps1 -Scenario fault_uav_link -MavPort 14560
#   .\run.ps1 -Gui -Show              # PyBullet GUI + live operator dashboard window
#   .\run.ps1 -Tests                  # fast coordination tests (no simulator)
param([ValidateSet("normal", "fault_ugv_link", "fault_uav_link")][string]$Scenario = "normal",
      [int]$MavPort = 14540, [double]$Seconds = 0, [switch]$Gui, [switch]$Show, [switch]$Tests)
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
. (Join-Path $here "..\scripts\env_micromamba.ps1") uavros
Set-Location $here
if ($Tests) {
    $env:PYTEST_DISABLE_PLUGIN_AUTOLOAD = "1"
    & $env:UAV_PYTHON -m pytest -q tests -p no:cacheprovider
    exit $LASTEXITCODE
}
$a = @("run_experiment.py", "--scenario", $Scenario, "--mav-port", $MavPort)
if ($Seconds -gt 0) { $a += @("--seconds", $Seconds) }
if ($Gui) { $a += "--gui" }
if ($Show) { $a += "--show" }
& $env:UAV_PYTHON -u @a
