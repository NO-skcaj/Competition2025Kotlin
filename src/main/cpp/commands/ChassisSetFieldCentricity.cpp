#include "commands/ChassisSetFieldCentricity.h"

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

