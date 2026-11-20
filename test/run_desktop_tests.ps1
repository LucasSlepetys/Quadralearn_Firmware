# Builds capture, joint-mapping, measured-pose and servo-command desktop tests.
# All executables run on Windows; this script never uploads to or opens the ESP32.
param([string]$Compiler = 'C:\msys64\mingw64\bin\g++.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $projectRoot '.pio\desktop_tests'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
$executable = Join-Path $buildDir 'whole_leg_test.exe'
$arguments = @('-std=c++11', '-Wall', '-Wextra', '-Werror', '-Wno-missing-field-initializers')
foreach ($includeDir in @('test\desktop', 'include', 'src', 'lib\ServoSupport\src')) {
    $arguments += '-I' + (Join-Path $projectRoot $includeDir)
}
foreach ($source in @(
    'test\desktop\test_whole_leg.cpp',
    'src\main.cpp',
    'test\desktop\mock_mcpwm.cpp',
    'src\Firmware\ServoPwm\servo_pwm.cpp',
    'src\Firmware\ServoBench\servo_bench.cpp',
    'src\Firmware\LegControl\leg_control.cpp',
    'src\Firmware\BenchLink\bench_link.cpp',
    'lib\ServoSupport\src\ServoCommand\servo_command.cpp',
    'src\Firmware\ServoFeedback\servo_feedback.cpp',
    'src\Firmware\CaptureLink\capture_link.cpp',
    'lib\ServoSupport\src\EndpointCapture\endpoint_capture.cpp',
    'lib\ServoSupport\src\ServoCalibration\servo_calibration.cpp',
    'lib\ServoSupport\src\Configuration\servo_configuration.cpp'
    'lib\ServoSupport\src\Configuration\installed_joint_configuration.cpp'
    'lib\ServoSupport\src\Configuration\robot_model.cpp'
    'lib\ServoSupport\src\JointMapping\joint_mapping.cpp'
    'lib\ServoSupport\src\ForwardKinematics\forward_kinematics.cpp'
    'lib\ServoSupport\src\MeasuredLegPose\measured_leg_pose.cpp'
)) { $arguments += Join-Path $projectRoot $source }
$arguments += @('-o', $executable)
& $Compiler @arguments
if ($LASTEXITCODE -ne 0) { throw 'Desktop test build failed.' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Desktop checks failed; see FAIL messages above.' }

# Separate executable: tests the installed measurements and both mapping directions.
$executable = Join-Path $buildDir 'installed_joint_configuration_test.exe'
$arguments = @('-std=c++11', '-Wall', '-Wextra', '-Werror', '-Wno-missing-field-initializers')
$arguments += '-I' + (Join-Path $projectRoot 'lib\ServoSupport\src')
foreach ($source in @(
    'test\desktop\test_installed_joint_configuration.cpp',
    'lib\ServoSupport\src\Configuration\installed_joint_configuration.cpp',
    'lib\ServoSupport\src\Configuration\servo_configuration.cpp',
    'lib\ServoSupport\src\JointMapping\joint_mapping.cpp',
    'lib\ServoSupport\src\ServoCalibration\servo_calibration.cpp'
)) { $arguments += Join-Path $projectRoot $source }
$arguments += @('-o', $executable)
& $Compiler @arguments
if ($LASTEXITCODE -ne 0) { throw 'Installed joint test build failed.' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Installed joint checks failed; see FAIL messages above.' }

# Pure measured-pose tests: captured voltages -> model angles -> FK points.
$executable = Join-Path $buildDir 'measured_leg_pose_test.exe'
$arguments = @('-std=c++11', '-Wall', '-Wextra', '-Werror', '-Wno-missing-field-initializers')
$arguments += '-I' + (Join-Path $projectRoot 'lib\ServoSupport\src')
foreach ($source in @(
    'test\desktop\test_measured_leg_pose.cpp',
    'lib\ServoSupport\src\MeasuredLegPose\measured_leg_pose.cpp',
    'lib\ServoSupport\src\ForwardKinematics\forward_kinematics.cpp',
    'lib\ServoSupport\src\Configuration\robot_model.cpp',
    'lib\ServoSupport\src\Configuration\installed_joint_configuration.cpp',
    'lib\ServoSupport\src\Configuration\servo_configuration.cpp',
    'lib\ServoSupport\src\JointMapping\joint_mapping.cpp',
    'lib\ServoSupport\src\ServoCalibration\servo_calibration.cpp',
    'lib\ServoSupport\src\EndpointCapture\endpoint_capture.cpp'
)) { $arguments += Join-Path $projectRoot $source }
$arguments += @('-o', $executable)
& $Compiler @arguments
if ($LASTEXITCODE -ne 0) { throw 'Measured pose test build failed.' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Measured pose checks failed; see FAIL messages above.' }

# ServoCommand is calculation-only: prepare pulse values without any PWM driver.
$executable = Join-Path $buildDir 'servo_command_test.exe'
$arguments = @('-std=c++11', '-Wall', '-Wextra', '-Werror', '-Wno-missing-field-initializers')
$arguments += '-I' + (Join-Path $projectRoot 'lib\ServoSupport\src')
foreach ($source in @(
    'test\desktop\test_servo_command.cpp',
    'lib\ServoSupport\src\ServoCommand\servo_command.cpp',
    'lib\ServoSupport\src\Configuration\installed_joint_configuration.cpp',
    'lib\ServoSupport\src\Configuration\servo_configuration.cpp',
    'lib\ServoSupport\src\JointMapping\joint_mapping.cpp',
    'lib\ServoSupport\src\ServoCalibration\servo_calibration.cpp'
)) { $arguments += Join-Path $projectRoot $source }
$arguments += @('-o', $executable)
& $Compiler @arguments
if ($LASTEXITCODE -ne 0) { throw 'Servo command test build failed.' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Servo command checks failed; see FAIL messages above.' }

# Optional bench checks are kept in their own script/source files.
& (Join-Path $PSScriptRoot 'run_bench_tests.ps1') -Compiler $Compiler

# Reusable three-joint coordinator tests, independent of the USB UI.
& (Join-Path $PSScriptRoot 'run_leg_control_tests.ps1') -Compiler $Compiler
