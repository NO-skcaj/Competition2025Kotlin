package org.team3824.frc2025.commands

import edu.wpi.first.wpilibj2.command.Command

import org.team3824.frc2025.subsystems.Drivetrain

// This whole command doesnt even work lmao why is it in the repo? is it used???

/// @brief Command to drive the robot the specified distance.
/// @param distance The distance to drive the robot.
/// @param speed The speed to perform the drive.
/// @param drivetrain The Drivetrains subsystem.
class ChassisDriveDistance(private val m_distance:        Double,
                           private val m_speed:           Double,
                           private val m_drivetrain:      Drivetrain) : Command()
{

    // Remember the field centric setting
    var m_fieldCentricity: Boolean = m_drivetrain.GetFieldCentricity()

    init {
        addRequirements(m_drivetrain)

        // Remember the field centric setting
        m_fieldCentricity = m_drivetrain.GetFieldCentricity()
    }

    override fun initialize()
    {
        // Do not use field coordinates
        m_drivetrain.SetFieldCentricity(false)
    }

    override fun execute()
    {
        m_drivetrain.Drive(m_speed, 0.0, 0.0)
    }

    override fun isFinished(): Boolean
    {
        return true;
    }

    override fun end(interrupted: Boolean)
    {
        // Stop the move
        m_drivetrain.Drive(0.0, 0.0, 0.0)

        // Restore the field centricity
        m_drivetrain.SetFieldCentricity(m_fieldCentricity)
    }
}