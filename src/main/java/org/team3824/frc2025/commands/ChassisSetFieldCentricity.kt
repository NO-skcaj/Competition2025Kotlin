package org.team3824.frc2025.commands

import edu.wpi.first.wpilibj2.command.Command

import org.team3824.frc2025.subsystems.Drivetrain

/// @brief Command to set the field centricity.
/// @param fieldCentric Boolean for the field centricity (true - field centric, false - robot centric)
/// @param drivetrain The Drivetrain subsystem.
class ChassisSetFieldCentricity(private var m_fieldCentric: Boolean,
                                private val m_drivetrain: Drivetrain) : Command()
{
    init {
        addRequirements(m_drivetrain)
    }

    /// @brief Called just before this Command runs the first time.
    override fun initialize()
    {
        m_drivetrain.SetFieldCentricity(m_fieldCentric);
    }

    /// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
    /// @return True is the command has completed.
    override fun isFinished(): Boolean
    {
        return true;
    }
};

