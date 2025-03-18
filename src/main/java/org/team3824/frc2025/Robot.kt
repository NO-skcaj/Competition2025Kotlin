package org.team3824.frc2025

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

import org.team3824.frc2025.RobotContainer
import org.team3824.frc2025.ControllerConstants

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : TimedRobot()
{
    // Instantiate the Robot container and get a pointer to the class
    private val m_container: RobotContainer = RobotContainer.getInstance();

    // Pointer to the autonomous command
    private var m_autonomousCommand : Command? = m_container.getAutonomousCommand();

    /// @brief Method is called every robot packet, no matter the mode.
    fun RobotPeriodic()
    {
        // Run the command scheduler
        CommandScheduler.getInstance().run();
    }
    
    /// @brief Method is called once each time the robot enters Disabled mode.
    fun DisabledInit()
    {
    
    }
    
    /// @brief Method is called periodically when the robot is disabled.
    fun DisabledPeriodic()
    {
    
    }
    
    /// @brief Method is called when switching to teleoperated mode.
    fun TeleopInit()
    {
        // Set the swerve wheels to zero
        m_container.setSwerveWheelAnglesToZero();
    
        // This makes sure that the autonomous stops running when teleop starts running.
        m_autonomousCommand?.cancel();
        m_autonomousCommand = null;
    }
    
    /// @brief Method is called periodically when the robot is in tleloperated mode.
    fun TeleopPeriodic()
    {
    
    }
    
    /// @brief Method is called when switching to autonomous mode.
    fun AutonomousInit()
    {
        // Set the swerve wheels to zero
        m_container.setSwerveWheelAnglesToZero();
    
        // Get the selected autonomous command
        m_autonomousCommand = m_container.getAutonomousCommand();
    
        // Determine if the chooser returned a pointer to a command
        // Schedule the autonomous command
        m_autonomousCommand?.schedule()
    }
    
    /// @brief Method is called periodically when the robot is in autonomous mode.
    fun AutonomousPeriodic()
    {
    
    }
    
    /// @brief Method is called when switching to test mode.
    fun TestInit()
    {
    
    }
    
    // This function is called periodically during test mode.
    fun TestPeriodic()
    {
    
    }
    
    /// @brief Method is called when starting in simulation mode.
    fun SimulationInit()
    {
    
    }
    
    /// @brief Method is called periodically when in simulation mode.
    fun SimulationPeriodic()
    {
    
    }
};