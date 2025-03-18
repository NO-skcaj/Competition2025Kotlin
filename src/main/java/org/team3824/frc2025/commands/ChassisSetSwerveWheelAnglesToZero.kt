package org.team3824.frc2025.commands

import edu.wpi.first.wpilibj2.command.Command
import org.team3824.frc2025.subsystems.Drivetrain

/// @brief Command to set the swerve wheels to the zero degree congiguration based on the absolute encode.
class ChassisSetSwerveWheelAnglesToZero(private val m_drivetrain: Drivetrain) : Command()
{
    init
    {
        addRequirements(m_drivetrain)
    }

    /// @brief Called repeatedly when this Command is scheduled to run.
    override fun execute()
    {
        m_drivetrain.SetWheelAnglesToZero()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean
    {

        return true;
    }

}