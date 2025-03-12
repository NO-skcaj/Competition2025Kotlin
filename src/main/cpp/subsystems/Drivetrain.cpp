#include "subsystems/DriveTrain.h"

/// @brief Class constructor for the DriveTrain subassembly.
/// Swerve Module Indexes:
///
///          Front
///       +---------+ ---
///       |[1]   [0]|  ^       0   Front Right
///       |         |  |       1   Front Left
///       |         | Length   2   Rear Left
///       |         |  |       3   Rear Right
///       |[2]   [3]|  v
///       +---------+ ---
///       |         |
///       |< Width >|
Drivetrain::Drivetrain()
{
    // Create the robot swerve modules
    m_swerveModule[0] = new SwerveModule(CanConstants::SwerveFrontRightDriveMotorCanId,
                                         CanConstants::SwerveFrontRightAngleMotorCanId,
                                         CanConstants::SwerveFrontRightAngleEncoderCanId);
    m_swerveModule[1] = new SwerveModule(CanConstants::SwerveFrontLeftDriveMotorCanId,
                                         CanConstants::SwerveFrontLeftAngleMotorCanId,
                                         CanConstants::SwerveFrontLeftAngleEncoderCanId);
    m_swerveModule[2] = new SwerveModule(CanConstants::SwerveRearLeftDriveMotorCanId,
                                         CanConstants::SwerveRearLeftAngleMotorCanId,
                                         CanConstants::SwerveRearLeftAngleEncoderCanId);
    m_swerveModule[3] = new SwerveModule(CanConstants::SwerveRearRightDriveMotorCanId,
                                         CanConstants::SwerveRearRightAngleMotorCanId,
                                         CanConstants::SwerveRearRightAngleEncoderCanId);
}

/// @brief This method will be called once periodically.
void Drivetrain::Periodic()
{
    frc::SmartDashboard::PutNumber("Gyro Angle",        GetHeading().value());
    frc::SmartDashboard::PutBoolean("Field Centricity", m_fieldCentricity);
}

/// @brief Method to drive the robot chassis.
/// @param forward The forward operater input.
/// @param strafe The strafe operater input.
/// @param angle The angle operater input.
void Drivetrain::Drive(double forward, double strafe, double angle)
{
    // forward = frc::SmartDashboard::GetNumber("Chassis Forward", 0.0);
    // strafe  = frc::SmartDashboard::GetNumber("Chassis Strafe",  0.0);
    // angle   = frc::SmartDashboard::GetNumber("Chassis Angle",   0.0);

    frc::SmartDashboard::PutNumber("Chassis Forward", forward);
    frc::SmartDashboard::PutNumber("Chassis Strafe",  strafe);
    frc::SmartDashboard::PutNumber("Chassis Angle",   angle);

    // Convert to field centric
    if (m_fieldCentricity)
        FieldCentricAngleConversion(&forward, &strafe, angle);

    // Create a wheel vector array for wheel vector calculations
    WheelVector wheelVector[ChassisConstants::NumberOfSwerveModules];

    // Calcualte the drive paramters
    CalculateSwerveModuleDriveAndAngle(forward, strafe, angle, wheelVector);

    frc::SmartDashboard::PutNumber("Front Right Drive", wheelVector[0].Drive);
    frc::SmartDashboard::PutNumber("Front Right Angle", wheelVector[0].Angle);
    frc::SmartDashboard::PutNumber("Front Left Drive",  wheelVector[1].Drive);
    frc::SmartDashboard::PutNumber("Front Left Angle",  wheelVector[1].Angle);
    frc::SmartDashboard::PutNumber("Rear Left Drive",   wheelVector[2].Drive);
    frc::SmartDashboard::PutNumber("Rear Left Angle",   wheelVector[2].Angle);
    frc::SmartDashboard::PutNumber("Rear Right Drive",  wheelVector[3].Drive);
    frc::SmartDashboard::PutNumber("Rear Right Angle",  wheelVector[3].Angle);

    // Update the swerve module
    for (auto swerveModuleIndex = 0; swerveModuleIndex < ChassisConstants::NumberOfSwerveModules; swerveModuleIndex++)
        m_swerveModule[swerveModuleIndex]->SetState(wheelVector[swerveModuleIndex]);

    // Read the swerve module angles and drive
    frc::SmartDashboard::PutNumber("Vector Front Right Drive", m_swerveModule[0]->GetWheelVector()->Drive);
    frc::SmartDashboard::PutNumber("Vector Front Right Angle", m_swerveModule[0]->GetWheelVector()->Angle);
    frc::SmartDashboard::PutNumber("Vector Front Left Drive",  m_swerveModule[1]->GetWheelVector()->Drive);
    frc::SmartDashboard::PutNumber("Vector Front Left Angle",  m_swerveModule[1]->GetWheelVector()->Angle);
    frc::SmartDashboard::PutNumber("Vector Rear Left Drive",   m_swerveModule[2]->GetWheelVector()->Drive);
    frc::SmartDashboard::PutNumber("Vector Rear Left Angle",   m_swerveModule[2]->GetWheelVector()->Angle);
    frc::SmartDashboard::PutNumber("Vector Rear Right Drive",  m_swerveModule[3]->GetWheelVector()->Drive);
    frc::SmartDashboard::PutNumber("Vector Rear Right Angle",  m_swerveModule[3]->GetWheelVector()->Angle);
}

/// @brief Method to set the robot control field centricity.
/// @param fieldCentric Boolean to indicate if the robor control should be field centric.
void Drivetrain::SetFieldCentricity(bool fieldCentric)
{
    // Set the field centric member variable
    m_fieldCentricity = fieldCentric;
}

/// @brief Method to set the field centricity.
/// @return The field centricity setting.
bool Drivetrain::GetFieldCentricity()
{
    // Return the field centricity setting
    return m_fieldCentricity;
}

/// <summary>
/// Method to get the specified swerve module wheel vector.
/// </summary>
/// <param name="swerveModuleIndex">The swerve module index.</param>
/// <param name="wheelVector">Variable to return the specified swerve module wheel vector.</param>
WheelVector* Drivetrain::GetSwerveModuleWheelVector(int swerveModuleIndex)
{
    // Get the specified swerve module wheel vector
    return m_swerveModule[swerveModuleIndex]->GetWheelVector();
}

/// <summary>
/// Methiod to convert the forward and strafe into field centric values based on the gyro angle.
///
/// Note: The drive motor range is 0.0 to 1.0 and the angle is in the range -180 to 180 degrees.
/// </summary>
/// <param name="forward">The forward power.</param>
/// <param name="strafe">The strafe (side) power.</param>
/// <param name="angle">The present robot angle relative to the field direction.</param>
void Drivetrain::FieldCentricAngleConversion(double *forward, double *strafe, double angle)
{
    // Copy the forward and strafe method parameters
    double forwardParameter = *forward;
    double strafeParamewter = *strafe;

    // Convert the angle from degrees to radians
    angle = angle * std::numbers::pi / 180;

    // Modify the input parameters for field centric control
    *forward =  forwardParameter * cos(angle) + strafeParamewter * sin(angle);
    *strafe  = -forwardParameter * sin(angle) + strafeParamewter * cos(angle);
}

/// <summary>
/// Method to output an array of speed and rotation values for each swerve module for a drive train given
/// the desired forward, strafe, and rotation.
///
/// See: "Derivation of Inverse Kinematics for Swerve.pdf" for calcuation details located at
///      https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
///
/// Swerve Module Indexes:
///
///          Front
///       +---------+ ---
///       |[1]   [0]|  ^       0   Front Right
///       |         |  |       1   Front Left
///       |         | Length   2   Rear Left
///       |         |  |       3   Rear Right
///       |[2]   [3]|  v
///       +---------+ ---
///       |         |
///       |< Width >|
///
/// </summary>
/// <param name="forward">positive value = forward movement,   negative value = backward movement</param>
/// <param name="strafe">positive value  = right direction,    negative value = left direction</param>
/// <param name="rotate">positive value  = clockwise rotation, negative value = counterclockwise rotation</param>
void Drivetrain::CalculateSwerveModuleDriveAndAngle(double forward, double strafe, double rotate, WheelVector wheelVector[])
{
    // Create intermediate values for the speed and angle calculations
    double A = strafe  - rotate * (ChassisConstants::ChassisLength / R);
    double B = strafe  + rotate * (ChassisConstants::ChassisLength / R);
    double C = forward - rotate * (ChassisConstants::ChassisWidth  / R);
    double D = forward + rotate * (ChassisConstants::ChassisWidth  / R);

    // Calculate the wheel angle and convert radians to degrees
    wheelVector[0].Angle = atan2(B, C) * 180 / std::numbers::pi;
    wheelVector[1].Angle = atan2(B, D) * 180 / std::numbers::pi;
    wheelVector[2].Angle = atan2(A, D) * 180 / std::numbers::pi;
    wheelVector[3].Angle = atan2(A, C) * 180 / std::numbers::pi;

    // Calculate the speed
    wheelVector[0].Drive = sqrt(B * B + C * C);
    wheelVector[1].Drive = sqrt(B * B + D * D);
    wheelVector[2].Drive = sqrt(A * A + D * D);
    wheelVector[3].Drive = sqrt(A * A + C * C);

    // Normalize the speed values
    NormalizeSpeed(wheelVector);
}

/// @brief Method to normalize the Drive values for a Swerve Module.
/// @param wheelVector Structure for returning the swerve module normalization for the drive motors.
void Drivetrain::NormalizeSpeed(WheelVector wheelVector[])
{
    // Determine the maximum speed
    double maxSpeed = wheelVector[0].Drive;
    for (auto wheelVectorIndex = 1; wheelVectorIndex < ChassisConstants::NumberOfSwerveModules; wheelVectorIndex++)
        if (wheelVector[wheelVectorIndex].Drive > maxSpeed)
            maxSpeed = wheelVector[wheelVectorIndex].Drive;

    // Normalizes speeds so they're within the ranges of -1 to 1
    if (maxSpeed > 1)
        for (auto wheelVectorIndex = 0; wheelVectorIndex < ChassisConstants::NumberOfSwerveModules; wheelVectorIndex++)
            wheelVector[wheelVectorIndex].Drive /= maxSpeed;
}

/// @brief Method to set the swerve wheel to the absoulute encoder angle then zero the PID controller angle.
void Drivetrain::SetWheelAnglesToZero()
{
    // Set the swerve wheel angles to zero
    m_swerveModule[SwerveConstants::FrontRightIndex]->SetWheelAngleToForward(SwerveConstants::FrontRightForwardAngle);
    m_swerveModule[SwerveConstants::FrontLeftIndex]->SetWheelAngleToForward(SwerveConstants::FrontLeftForwardAngle);
    m_swerveModule[SwerveConstants::RearRightIndex]->SetWheelAngleToForward(SwerveConstants::RearRightForwardAngle);
    m_swerveModule[SwerveConstants::RearLeftIndex]->SetWheelAngleToForward(SwerveConstants::RearLeftForwardAngle);
}

/// @brief Method to get the robot heading.
/// @return The robot heading.
units::degree_t Drivetrain::GetHeading()
{
    // Return the robot heading
    return (units::degree_t) m_navx.GetAngle();
}

