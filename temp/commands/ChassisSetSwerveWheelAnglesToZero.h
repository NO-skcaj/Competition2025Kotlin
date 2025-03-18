#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class ChassisSetSwerveWheelAnglesToZero : public frc2::CommandHelper<frc2::Command, ChassisSetSwerveWheelAnglesToZero>
{
    public:

        ChassisSetSwerveWheelAnglesToZero(Drivetrain *drivetrain);

        void Execute()    override;
        bool IsFinished() override;

    private:

        Drivetrain *m_drivetrain;  // Pointer to the chassis set the swerve wheel angles to zero
};

/// @brief Command to set the swerve wheels to the zero degree congiguration based on the absolute encode.
ChassisSetSwerveWheelAnglesToZero::ChassisSetSwerveWheelAnglesToZero(Drivetrain *drivetrain) : m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisSetSwerveWheelAnglesToZero");

    // Declare subsystem dependencies
    AddRequirements({m_drivetrain});
}


/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisSetSwerveWheelAnglesToZero::Execute()
{
    // Set the swerve wheel angles to zero
    m_drivetrain->SetWheelAnglesToZero();
}


// Returns true when the command should end.
bool ChassisSetSwerveWheelAnglesToZero::IsFinished()
{
    // Execute only runs once
    return true;
}

