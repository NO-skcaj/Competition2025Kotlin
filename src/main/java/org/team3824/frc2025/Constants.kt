package org.team3824.frc2025


object CanConstants {
    const val CanBus                            = "rio";

    const val SwerveFrontRightDriveMotorCanId   =  3;
    const val SwerveFrontRightAngleMotorCanId   =  2;
    const val SwerveFrontRightAngleEncoderCanId = 12;

    const val SwerveFrontLeftDriveMotorCanId    =  6;
    const val SwerveFrontLeftAngleMotorCanId    =  5;
    const val SwerveFrontLeftAngleEncoderCanId  =  4;

    const val SwerveRearLeftDriveMotorCanId     =  9;
    const val SwerveRearLeftAngleMotorCanId     =  8;
    const val SwerveRearLeftAngleEncoderCanId   =  7;

    const val SwerveRearRightDriveMotorCanId    = 11;
    const val SwerveRearRightAngleMotorCanId    = 10;
    const val SwerveRearRightAngleEncoderCanId  = 13;

    const val ElevatorMotorCanId                 = 20;
}

object ChassisConstants
{
    const val NumberOfSwerveModules = 4;

    const val ChassisLength = 30;  // Inches
    const val ChassisWidth = 30;

    const val MotorConfigurationAttempts = 5;

    const val SwerveDriveMaxAmperage = 60;

    const val SwerveAngleMaxAmperage = 30;

    const val SwerveMotorRevolutions = 21.5;     // The number of motor revolutions per wheel revolutions

    const val SwerveDegreesToMotorRevolutions =
        180.0 / (SwerveMotorRevolutions / 2.0);  // Degrees to motor revolutions
}

object SwerveConstants
{
    const val FrontRightIndex = 0;
    const val FrontLeftIndex = 1;
    const val RearRightIndex = 3;
    const val RearLeftIndex = 2;

    const val FrontRightForwardAngle = 0.301 * 360; // degrees
    const val FrontLeftForwardAngle = -0.464 * 360;
    const val RearRightForwardAngle = -0.064 * 360;
    const val RearLeftForwardAngle = -0.022 * 360;

    const val P = 0.025;
    const val I = 0.000;
    const val D = 0.010;
}

object ElevatorConstants
{
    val S = 0.25;             // Static Friction: Add [voltage] output to overcome static friction
    const val V = 0.12;       // Velocity:        A velocity target of 1 rps results in [voltage] output
    const val A = 0.01;       // Acceleration:    An acceleration of 1 rps/s requires 0.01 V output
    const val P = 60.0;       // Proportional:    A position error of 0.2 rotations results in 12 V output
    const val I = 0.0;        // Integral:        No output for integrated error
    const val D = 0.5;        // Differential     A velocity error of 1 rps results in 0.5 V output

    const val SensorToMechanismRatio = 12.8;             // 12.8 rotor rotations per mechanism rotation

    const val MotionMagicCruiseVelocity = 5.0;            // 5 (mechanism) rotations per second cruise
    const val MotionMagicAcceleration = 10.0;   // Take approximately 0.5 seconds to reach max vel
    const val MotionMagicJerk = 100.0;  // Take approximately 0.1 seconds to reach max accel

    const val PositionToTurnsConversionFactor = 1.0;

    const val MotorConfigurationAttempts = 5;
}

object ControllerConstants
{
    const val DriverControllerUsbPort = 0;
    const val JoystickOperatorUsbPort = 1;

    const val JoystickForwardIndex = 1;
    const val JoystickStrafeIndex = 0;
    const val JoystickAngleIndex = 2;  // 4 for xbox controller, 2 for extreme 3d controller(stick controller)

    const val JoystickAngleDeadZone = 0.3;
    const val JoystickDeadZone = 0.1;

    const val ExponentForward = 2.0;
    const val ExponentStrafe = 2.0;
    const val ExponentAngle = 2.0;
}

object Extreme3DConstants
{
    const val HandleLowerLeft = 3;
    const val HandleLowerRight = 4;
}

object XBoxConstants
{
    const val A = 1;
    const val B = 2;
    const val X = 3;
    const val Y = 4;
    const val LeftBumper = 5;
    const val RightBumper = 6;
    const val Back = 7;
    const val Start = 8;
    const val LeftStickButton = 9;
    const val RightStickButton = 10;

    const val Pov_0 = 0;
    const val Pov_45 = 45;
    const val Pov_90 = 90;
    const val Pov_135 = 135;
    const val Pov_180 = 180;
    const val Pov_225 = 225;
    const val Pov_270 = 270;
    const val Pov_315 = 315;
}

object ApriltagConstants
{
    // Magic camera values:
    // Source: https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
    const val CameraWidthInPixels = 699.3778103158814;
    const val CameraHeightInPixels = 677.7161226393544;
    const val CameraCenterXInPixels = 345.6059345433618;
    const val CameraCenterYInPixels = 207.12741326228522;

    const val CameraResolutionWidth = 640;
    const val CameraResolutionHeight = 480;

    const val AprilTagLineWitdh = 2;
    const val NumberOfAprilTagCorners = 4;
    const val NumberOfBitsCorrected = 1;

    const val LengthOfTagsInches = 6.5;
}

object LedConstants
{
    const val Length: Int     = 40;  // The length of the LED string
    const val PwmPort: Int    = 5;
    const val Brightness: Int = 2; // DIVIDE by this number, effectively / 2

    const val Red:   Int = 255;
    const val Green: Int = 255;
    const val Blue:  Int = 255;

    const val StrobeDelay = 20;  // The delay between strobe flashes
    const val HvaDelay = 20;  // The delay between HVA color changes
}