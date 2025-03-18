package org.team3824.frc2025.commands

import edu.wpi.first.wpilibj2.command.Command
import org.team3824.frc2025.subsystems.Drivetrain

import edu.wpi.first.wpilibj.RobotController.getTime
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Time

/// @brief Command to drive the robot the specified time.
/// @param time The time to drive the robot.
/// @param speed The speed to perform the drive.
/// @param drivetrain The Drivetrains subsystem.
class ChassisDriveTime(private val m_time: Time,
                       private val m_speed: Double,
                       private val m_drivetrain: Drivetrain) : Command()
{

    var m_fieldCentricity: Boolean = true  // The field centricity flag
    var m_startTime: Time = Seconds.of(0.0) // The start of the drive time

    init {
        addRequirements(m_drivetrain);
    }

    /// @brief Called just before this Command runs the first time.
    override fun initialize()
    {
        // Remember the field centric setting
        m_fieldCentricity = m_drivetrain.GetFieldCentricity();

        // Do not use field coordinates
        m_drivetrain.SetFieldCentricity(false);

        // Get the start time
        m_startTime = Units.Seconds.of(getTime().toDouble())

    }

    /// @brief Called repeatedly when this Command is scheduled to run.
    override fun execute()
    {
        m_drivetrain.Drive(m_speed, 0.0, 0.0);
    }

    /// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
    /// @return True is the command has completed.
    override fun isFinished(): Boolean
    {
        // Determine if the sequence is complete
        if (Units.Seconds.of(getTime().toDouble()) - m_startTime > m_time)
            return true;

        // Still driving
        return false;
    }

    /// @brief Called once after isFinished returns true.
    /// @param interrupted Indicated that the command was interrupted.
    override fun end(interrupted: Boolean)
    {
        // Stop the move
        m_drivetrain.Drive(0.0, 0.0, 0.0);

        // Restore the field centricity
        m_drivetrain.SetFieldCentricity(m_fieldCentricity);

    }
}