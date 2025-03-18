package org.team3824.frc2025

import kotlin.math.pow

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.POVButton

import org.team3824.frc2025.subsystems.*
import org.team3824.frc2025.commands.*
import org.team3824.frc2025.CanConstants // we do this

/**
 * Class to instantiate the robot subsystems and commands along with the operator controls
 */
// REDO THIS TO MAKE RobotContainer AN OBJECT, NOT A CLASS
class RobotContainer private constructor() {
    // Joysticks
    private val m_driverController = Joystick(ControllerConstants.DriverControllerUsbPort)
    private val m_operatorController = XboxController(ControllerConstants.JoystickOperatorUsbPort)

    // Autonomous command chooser
    private val m_autonomousChooser = SendableChooser<Command>()

    // Instantiate the robot subsystems
    private val m_aprilTags = AprilTags
    private val m_drivetrain = org.team3824.frc2025.subsystems.Drivetrain
    private val m_elevator = org.team3824.frc2025.subsystems.Elevator
    private val m_leds = LEDs

    init {
        SmartDashboard.putData("ChassisDrive: Stop",       ChassisDriveDistance(0.0, 0.0, m_drivetrain))
        SmartDashboard.putData("DriveDistance: OneMeter",  ChassisDriveDistance(1.0, 0.5, m_drivetrain))
        SmartDashboard.putData("DriveDistance: TwoMeters", ChassisDriveDistance(2.0, 0.5, m_drivetrain))

        // Bind the joystick controls to the robot commands
        configureButtonBindings()

        // Configure the autonomous command chooser
        m_autonomousChooser.setDefaultOption("Do Nothing", AutonomousDoNothing())
        m_autonomousChooser.addOption("Drive Forward OneMeter", ChassisDriveDistance(1.0, 0.5,  m_drivetrain))
        m_autonomousChooser.addOption("Drive Forward TwoMeters", ChassisDriveDistance(2.0, 0.5, m_drivetrain))

        // Send the autonomous mode chooser to the SmartDashboard
        SmartDashboard.putData("Autonomous Mode", m_autonomousChooser)

        // Set the default commands for the subsystems
        m_drivetrain.defaultCommand = ChassisDrive(
            { forward() },
            { strafe() },
            { angle() },
            m_drivetrain
        )

        m_leds.defaultCommand = SetLeds(LEDMode.Off, m_leds)
    }

    /**
     * Method to bind the joystick controls to the robot commands
     */
    private fun configureButtonBindings() {
        // Bind the driver controller buttons to the robot commands
        val fieldCentricOn = JoystickButton(m_driverController, Extreme3DConstants.HandleLowerLeft)
        fieldCentricOn.onTrue(ChassisSetFieldCentricity(true, m_drivetrain)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))

        val fieldCentricOff = JoystickButton(m_driverController, Extreme3DConstants.HandleLowerRight)
        fieldCentricOff.onTrue(ChassisSetFieldCentricity(false, m_drivetrain)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))

        // Bind the operator controller buttons to the robot commands
        val setLedsOff = JoystickButton(m_operatorController, XBoxConstants.LeftStickButton)
        setLedsOff.onTrue(SetLeds(LEDMode.Off, m_leds)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))

        val setLedsStrobe = JoystickButton(m_operatorController, XBoxConstants.RightStickButton)
        setLedsStrobe.onTrue(SetLeds(LEDMode.Strobe, m_leds)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))

        val setLedsShootingAnimation = JoystickButton(m_operatorController, XBoxConstants.A)
        setLedsShootingAnimation.onTrue(SetLeds(LEDMode.ShootingAnimation, m_leds)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))

        val setLedsSolidGreen = POVButton(m_operatorController, XBoxConstants.Pov_0)
        setLedsSolidGreen.onTrue(SetLeds(LEDMode.SolidGreen, m_leds)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))

        val setLedsSolidRed = POVButton(m_operatorController, XBoxConstants.Pov_90)
        setLedsSolidRed.onTrue(SetLeds(LEDMode.SolidRed, m_leds)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))

        val setLedsHvaColors = POVButton(m_operatorController, XBoxConstants.Pov_180)
        setLedsHvaColors.onTrue(SetLeds(LEDMode.HvaColors, m_leds)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))

        val setLedsRainbow = POVButton(m_operatorController, XBoxConstants.Pov_270)
        setLedsRainbow.onTrue(SetLeds(LEDMode.Rainbow, m_leds)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))
    }

    /**
     * Method to return the driver joystick
     * @return The driver joystick
     */
    fun getDriverController(): Joystick {
        return m_driverController
    }

    /**
     * Method to return the operator controller
     * @return The operator controller
     */
    fun getOperatorController(): XboxController {
        return m_operatorController
    }

    /**
     * Method to return the autonomous command
     * @return The autonomous command
     */
    fun getAutonomousCommand(): Command {
        // The selected command will be run in autonomous
        return m_autonomousChooser.selected
    }

    /**
     * Method to set the swerve wheels to zero degrees based on the absolute encoder
     */
    fun setSwerveWheelAnglesToZero() {
        // Create the command to set the swerve wheel angles to zero based on the absolute encoder
        val command = ChassisSetSwerveWheelAnglesToZero(m_drivetrain)

        // Execute the command
        command.execute()
    }

    /**
     * Method to return the forward joystick value
     * @return The forward joystick value
     */
    fun forward(): Double {
        // Get the forward joystick setting
        var joystickForward = getDriverController().getRawAxis(ControllerConstants.JoystickForwardIndex)

        // Use exponential function to calculate the forward value for better slow speed control
        joystickForward = getExponentialValue(joystickForward, ControllerConstants.JoystickDeadZone, ControllerConstants.ExponentForward)

        // Return the x speed
        return -joystickForward
    }

    /**
     * Method to return the strafe joystick value
     * @return The strafe joystick value
     */
    fun strafe(): Double {
        // Get the strafe joystick setting
        var joystickStrafe = getDriverController().getRawAxis(ControllerConstants.JoystickStrafeIndex)

        // Use exponential function to calculate the forward value for better slow speed control
        joystickStrafe = getExponentialValue(joystickStrafe, ControllerConstants.JoystickDeadZone, ControllerConstants.ExponentStrafe)

        // Return the y speed
        return joystickStrafe
    }

    /**
     * Method to return the angle joystick value
     * @return The angle joystick value
     */
    fun angle(): Double {
        // Get the angle joystick setting
        var joystickAngle = getDriverController().getRawAxis(ControllerConstants.JoystickAngleIndex)

        // Use exponential function to calculate the forward value for better slow speed control
        if (joystickAngle != 0.0) {
            joystickAngle = getExponentialValue(joystickAngle, ControllerConstants.JoystickAngleDeadZone, ControllerConstants.ExponentAngle)
        }

        // Return the rotation speed
        return joystickAngle
    }

    /**
     * Method to convert a joystick value from -1.0 to 1.0 to exponential mode
     * @param joystickValue The raw joystick value
     * @param deadZone The dead zone value
     * @param exponent The exponential value
     * @return The resultant exponential value
     */
    private fun getExponentialValue(joystickValue: Double, deadZone: Double, exponent: Double): Double {
        var direction = 1
        var output: Double // Default constructor is 0.0
        var value = joystickValue

        // Ignore joystick input if it's too small
        if (value > -deadZone && value < deadZone) {
            return 0.0
        }

        // Direction is either 1 or -1, based on joystick value
        if (value < 0.0) {
            // Reverse the direction and make the joystick value positive
            direction = -1
            value *= -1.0
        }

        // Plug joystick value into exponential function
        output = direction * value.pow(exponent)

        // Ensure the range of the output
        if (output < -1.0) output = -1.0
        if (output > 1.0) output = 1.0

        // Return the calculated value
        return output
    }

    companion object {
        // Singleton reference to the class
        private var m_robotContainer: RobotContainer? = null

        /**
         * Method to return a reference to the RobotContainer class
         * @return Reference to the RobotContainer class
         */
        fun getInstance(): RobotContainer {
            // Determine if the class has already been instantiated
            if (m_robotContainer == null) {
                // Instantiate the class
                m_robotContainer = RobotContainer()
            }

            // Return the class reference
            return m_robotContainer!!
        }
    }
}