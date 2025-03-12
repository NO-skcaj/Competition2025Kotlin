#include "commands/ChassisTurnAngle.h"

/// @brief Command to rotate the chassis to the specified angle.
/// @param angle The desire robot angle.
/// @param speed The speed to move the chiaais.
/// @param timeoutTime The timeout time for the rotation.
/// @param drivetrain The Drivetrain subsystem.
ChassisTurnAngle::ChassisTurnAngle(units::angle::degrees angle, double speed, units::time::second_t timeoutTime, Drivetrain *drivetrain) :
                                   m_angle(angle), m_speed(speed), m_timeoutTime(timeoutTime), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisTurnAngle");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}

/// @brief Called just before this Command runs the first time.
void ChassisTurnAngle::Initialize()
{
    // Get the start time
    m_startTime = frc::GetTime();
}

/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisTurnAngle::Execute()
{

}


/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ChassisTurnAngle::IsFinished()
{
    // Determine if the sequence is complete
    if (frc::GetTime() - m_startTime > m_timeoutTime)
        return true;

    // Still driving
    return false;
}

/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void ChassisTurnAngle::End(bool interrupted)
{
    // Stop the move
    m_drivetrain->Drive(0.0, 0.0, 0.0);
}

