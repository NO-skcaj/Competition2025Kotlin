package org.team3824.frc2025.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.AngularVelocity

import edu.wpi.first.wpilibj2.command.Command

import org.team3824.frc2025.subsystems.Drivetrain

/// @brief Command to rotate the chassis to the specified angle.
/// @param angle The desire robot angle.
/// @param speed The speed to move the chiaais.
/// @param timeoutTime The timeout time for the rotation.
/// @param drivetrain The Drivetrain subsystem.
class ChassisTurnAngle(private val m_angle: Angle,
                       private val m_drivetrain: Drivetrain) : Command()
{
    private val PID: PIDController = PIDController(1.0, 0.00, 0.01)

    init
    {
        addRequirements(m_drivetrain)
    }

    /// @brief Called just before this Command runs the first time.
    override fun initialize()
    {

    }

    override fun execute()
    {
        // Use PID to get to setpoint without overshooting
        m_drivetrain.Drive(0.0, 0.0, PID.calculate(m_drivetrain.GetHeading().magnitude(), m_angle.magnitude()));
    }

    override fun end(interrupted: Boolean)
    {
        m_drivetrain.Drive(0.0, 0.0, 0.0);
    }

    override fun isFinished(): Boolean
    {
        if (m_drivetrain.GetHeading() + Degrees.of(2.5) > m_angle && m_drivetrain.GetHeading() - Degrees.of(2.5) < m_angle)
            return true

        // Still driving
        return false

    }



}