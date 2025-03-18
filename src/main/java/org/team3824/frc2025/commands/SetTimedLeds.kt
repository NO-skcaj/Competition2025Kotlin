package org.team3824.frc2025.commands

import edu.wpi.first.units.TimeUnit
import edu.wpi.first.wpilibj2.command.Command

import edu.wpi.first.wpilibj.RobotController.getTime
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Time

import org.team3824.frc2025.subsystems.LEDs
import org.team3824.frc2025.subsystems.LEDMode

/// @brief Command to set the LED mode.
/// @param Mode The LED mode.
/// @param m_leds The LED subsystem.
class SetTimedLeds(private val m_mode: LEDMode,
                   private val m_time: Time,
                   private val m_leds: LEDs): Command()
{
    private var m_startTime: Time = Seconds.of(0.0)

    /// @brief add requirements
    init
    {
        // Declare subsystem dependencies
        addRequirements(m_leds)
    }

    /// @brief Called just before this Command runs the first time.
    override fun initialize()
    {
        m_leds.SetMode(m_mode);

        m_startTime = Units.Seconds.of(getTime().toDouble())
    }

    /// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
    /// @return True is the command has completed.
    override fun isFinished(): Boolean
    {
        // Determine if the LED sequence is complete
        if (Units.Seconds.of(getTime().toDouble()) - m_startTime > m_time)
            return true;

        return false;
    }

    /// @brief Called once after isFinished returns true.
    /// @param interrupted Indicated that the command was interrupted.
    override fun end(interrupted: Boolean)
    {
        m_leds.SetMode(LEDMode.Off)
    }

    /// @brief Indicates if the command runs when the robot is disabled.
    /// @return True is the command should run when the robot is disabled.
    override fun runsWhenDisabled(): Boolean
    {
        return true;
    }
}