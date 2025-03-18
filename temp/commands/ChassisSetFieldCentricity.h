#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class ChassisSetFieldCentricity : public frc2::CommandHelper<frc2::Command, ChassisSetFieldCentricity>
{
    public:

        explicit ChassisSetFieldCentricity(bool fieldCentric, Drivetrain *drivetrain);

        void Initialize() override;
        bool IsFinished() override;
        
    private:

        bool        m_fieldCentric;  // The field centricity flag
        Drivetrain *m_drivetrain;    // Pointer to the chassis set field centricity class
};

/// @brief Command to set the field centricity.
/// @param fieldCentric Boolean for the field centricity (true - field centric, false - robot centric)
/// @param drivetrain The Drivetrain subsystem.
ChassisSetFieldCentricity::ChassisSetFieldCentricity(bool fieldCentric, Drivetrain *drivetrain) : m_fieldCentric(fieldCentric), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisSetFieldCentricity");

    // Declare subsystem dependencies
    AddRequirements({m_drivetrain});
}


/// @brief Called just before this Command runs the first time.
void ChassisSetFieldCentricity::Initialize()
{
    // Set the field centricity
    m_drivetrain->SetFieldCentricity(m_fieldCentric);
}


/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ChassisSetFieldCentricity::IsFinished()
{
    // Execute only runs once
    return true;
}

