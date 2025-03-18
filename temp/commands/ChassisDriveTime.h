#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

#include "Constants.h"

class ChassisDriveTime : public frc2::CommandHelper<frc2::Command, ChassisDriveTime>
{
    public:

        explicit ChassisDriveTime(units::second_t time, double speed, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        units::second_t m_time;             // The length of time that the chass will drive
        double          m_speed;            // The speed of the chassis
        Drivetrain     *m_drivetrain;       // The drivetrain subsystem

        bool            m_fieldCentricity;  // The field centricity flag
        units::second_t m_startTime;        // The start of the drive time
};

/// @brief Command to drive the robot the specified time.
/// @param time The time to drive the robot.
/// @param speed The speed to perform the drive.
/// @param drivetrain The Drivetrains subsystem.
ChassisDriveTime::ChassisDriveTime(units::second_t time, double speed, Drivetrain *drivetrain) : m_time(time), m_speed(speed), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveTime");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}


/// @brief Called just before this Command runs the first time.
void ChassisDriveTime::Initialize()
{
    // Remember the field centric setting
    m_fieldCentricity = m_drivetrain->GetFieldCentricity();

    // Do not use field coordinates
    m_drivetrain->SetFieldCentricity(false);

    // Get the start time
    m_startTime = frc::GetTime();
}


/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDriveTime::Execute()
{
    // Start driving
    m_drivetrain->Drive(m_speed, 0.0, 0.0);
}


/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ChassisDriveTime::IsFinished()
{
    // Determine if the sequence is complete
    if (frc::GetTime() - m_startTime > m_time)
        return true;

    // Still driving
    return false;
}


/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void ChassisDriveTime::End(bool interrupted)
{
    // Stop the move
    m_drivetrain->Drive(0.0, 0.0, 0.0);

    // Restore the field centricity
    m_drivetrain->SetFieldCentricity(m_fieldCentricity);
}

