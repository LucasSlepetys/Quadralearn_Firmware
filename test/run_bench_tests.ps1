# Optional bench tests. No serial port is opened and nothing is uploaded.
param([string]$Compiler = 'C:\msys64\mingw64\bin\g++.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $projectRoot '.pio\desktop_tests'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
$arguments = @('-std=c++11', '-Wall', '-Wextra', '-Werror', '-Wno-missing-field-initializers')
foreach ($includeDir in @('test\desktop', 'include', 'src', 'lib\ServoSupport\src')) {
    $arguments += '-I' + (Join-Path $projectRoot $includeDir)
}
foreach ($source in @(
    'test\desktop\test_servo_bench.cpp',
    'test\desktop\mock_mcpwm.cpp',
    'src\main.cpp',
    'src\Firmware\ServoPwm\servo_pwm.cpp',
    'src\Firmware\ServoBench\servo_bench.cpp',
    'src\Firmware\LegControl\leg_control.cpp',
    'src\Firmware\BenchLink\bench_link.cpp',
    'src\Firmware\ServoFeedback\servo_feedback.cpp',
    'src\Firmware\CaptureLink\capture_link.cpp',
    'lib\ServoSupport\src\EndpointCapture\endpoint_capture.cpp',
    'lib\ServoSupport\src\ServoCalibration\servo_calibration.cpp',
    'lib\ServoSupport\src\ServoCommand\servo_command.cpp',
    'lib\ServoSupport\src\Configuration\servo_configuration.cpp',
    'lib\ServoSupport\src\Configuration\installed_joint_configuration.cpp',
    'lib\ServoSupport\src\Configuration\robot_model.cpp',
    'lib\ServoSupport\src\JointMapping\joint_mapping.cpp',
    'lib\ServoSupport\src\ForwardKinematics\forward_kinematics.cpp',
    'lib\ServoSupport\src\MeasuredLegPose\measured_leg_pose.cpp'
)) { $arguments += Join-Path $projectRoot $source }
$executable = Join-Path $buildDir 'servo_bench_test.exe'
$arguments += @('-o', $executable)
& $Compiler @arguments
if ($LASTEXITCODE -ne 0) { throw 'Servo bench test build failed.' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Servo bench checks failed; see FAIL messages above.' }
