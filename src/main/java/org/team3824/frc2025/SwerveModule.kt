package org.team3824.frc2025;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units.Volts
import org.team3824.frc2025.CanConstants.CanBus
import kotlin.math.IEEErem


/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
class SwerveModule(val driveMotorCanId: Int, val angleMotorCanId: Int, val angleEncoderCanId: Int)
{

    // Swerve vector struture (drive and Angle)
    var m_wheelVector: WheelVector = WheelVector(0.0, 0.0)

    // Swerve drive motor
    var m_voltageOut: VoltageOut = VoltageOut(Volts.zero())  // Controller mode is VoltageOut

    // Swerve angle motor, encoder and PID controller
    val m_driveMotor           = TalonFX                  (driveMotorCanId,   CanBus);
    val m_angleMotor           = SparkMax                 (angleMotorCanId,   SparkLowLevel.MotorType.kBrushless);
    val m_angleAbsoluteEncoder = CANcoder                 (angleEncoderCanId, CanBus);
    val m_angleEncoder         = m_angleMotor.encoder;
    val m_pidController        = m_angleMotor.closedLoopController;

    init {

        // Configure the drive and angle motors
        ConfiguredriveMotor();
        ConfigureAngleMotor();
    }

    /// @brief Set the swerve module angle and motor power.
    /// @param vector The wheel vector (angle and drive).
    fun SetState(vector: WheelVector)
    {
        // Do not change the angle if the wheel is not driven
        if (vector.drive > 0.01 || vector.drive < -0.01)
        {
            // Optimize the serve module vector to minimize wheel rotation on change of diretion
            OptimizeWheelAngle(vector, m_wheelVector)

            // Set the angle motor PID set angle
            m_pidController.setReference(m_wheelVector.angle, SparkBase.ControlType.kPosition)
        }
        else
        {
            // Ensure the drive motor is disabled
            m_wheelVector.drive = 0.0
        }

        // Set the drive motor voltage
        m_driveMotor.setVoltage(m_voltageOut.withOutput(m_wheelVector.drive * 12).Output)
    }

    /// @brief Method to read the absolute encode in Degrees.
    /// @return The absolute angle value in degrees.
    fun GetAbsoluteAngle(): Double
    {
        // The GetAbsolutePosition() method returns a value from -1 to 1
        val encoderValue: Double = m_angleAbsoluteEncoder.getAbsolutePosition().valueAsDouble

        // To convert to degrees
        return encoderValue * 360
    }

    /// <summary>
    /// Method to get the swerve module wheel vector.
    /// </summary>
    /// <param name="wheelVector">Variable to return the swerve module wheel vector.</param>
    fun GetWheelVector(): WheelVector = m_wheelVector;

    /// @brief Method to get the swerve module angle encoder position.
    /// @return The angle encoder position
    fun GetSwerveAngle(): Double = m_angleEncoder.getPosition();

    /// @brief Method to set the swerve wheel encoder to the forward angle.
    /// @param angle The absolute angle for the forward direction in degrees.
    fun SetWheelAngleToForward(forwardAngle: Double)
    {
        // Set the motor angle encoder position to the forward direction
        m_angleMotor.encoder.setPosition(forwardAngle - GetAbsoluteAngle());

        // Ensure the PID controller set angle is zero (forward)
        m_pidController.setReference(0.0, SparkBase.ControlType.kPosition)
    }

    // Private methods

    /// @brief Method to configure the drive motor.
    private fun ConfiguredriveMotor()
    {
        // Create the drive motor configuration
        val driveMotorConfiguration: TalonFXConfiguration = TalonFXConfiguration();

        // Add the Motor Output section settings
        val motorOutputConfigs: MotorOutputConfigs = driveMotorConfiguration.MotorOutput
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast

        // Add the Current Limits section settings
        val currentLimitsConfigs: CurrentLimitsConfigs = driveMotorConfiguration.CurrentLimits;
        currentLimitsConfigs.StatorCurrentLimit       = ChassisConstants.SwerveDriveMaxAmperage.toDouble().toDouble()
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        // Apply the configuration to the drive motor
        for ( attempt in 1..ChassisConstants.MotorConfigurationAttempts)
        {
            // Check if the configuration was successful
            if (m_driveMotor.configurator.apply(driveMotorConfiguration).isOK) break
        }

        // Determine if the last configuration load was successful
//        if (!status.isOK())
//            std::cout << "***** ERROR: Could not configure swerve motor. Error: " << status.GetName() << std::endl
    }

    /// @brief Method to configure the angle motor and encoder.
    private fun ConfigureAngleMotor()
    {
        // Configure the angle motor
        val sparkBaseConfig: SparkMaxConfig = SparkMaxConfig();
        sparkBaseConfig.idleMode(SparkBaseConfig.IdleMode.kBrake)
        sparkBaseConfig.secondaryCurrentLimit(ChassisConstants.SwerveAngleMaxAmperage.toDouble())
        sparkBaseConfig.encoder.positionConversionFactor(ChassisConstants.SwerveDegreesToMotorRevolutions).velocityConversionFactor(
            1.0
        )
        sparkBaseConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                        .pid(SwerveConstants.P, SwerveConstants.I, SwerveConstants.D)
        m_angleMotor.configure(sparkBaseConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    /// @brief Method to determine the optimal swerve module wheel angle given the desired wheel vector.
    /// @brief Note: The swerve module angle is not restricted to -180 to 180 degrees, but is the actual module angle.
    /// @param wheelVector The target swerve module wheel drive power and angle.
    private fun OptimizeWheelAngle(targetWheelVector: WheelVector, wheelVector: WheelVector)
    {
        var driveDirection = 1.0  // Forward direction

        // Convert the present wheel angle to the same hemi-sphere as the target wheel angle
        val workingAngle = ConvertAngleToTargetRange(wheelVector)

        // Determine the angle between the past and desired swerve angle
        var angleDifference = targetWheelVector.angle - workingAngle

        // Determine if the angle is greater that obtuse (greater that 180 degrees)
        if (angleDifference > 180.0)
        {
            // Get the accute angle and change wheel correction direction
            angleDifference = angleDifference - 180.0
            driveDirection *= -1.0
        }
        else if (angleDifference < -180.0)
        {
            // Get the accute angle and change wheel correction direction
            angleDifference = angleDifference + 180.0
            driveDirection *= -1.0
        }

        // Minimize the wheel rotation
        if (angleDifference > 90.0)
        {
            // Get the minimized wheel angle and change wheel correction direction
            angleDifference = angleDifference - 180.0

            // Reverse the drive direction
            driveDirection *= -1.0
        }
        else if (angleDifference < -90.0)
        {
            // Get the minimized wheel angle and change wheel correction direction
            angleDifference = angleDifference + 180.0

            // Reverse the drive direction
            driveDirection *= -1.0
        }

        // Set the wheel vector to the target
        wheelVector.angle += angleDifference
        wheelVector.drive  = targetWheelVector.drive * driveDirection
    }

    /// <summary>
    /// Convert any angle to the range -180 to 180 degrees.
    /// </summary>
    /// <param name="angle">The angle to convert.</param>
    /// <returns>The angle represented from -180 to 180 degrees.</returns>
    private fun ConvertAngleToTargetRange(wheelVector: WheelVector): Double
    {
        // Get the angle between -360 and 360
        var angle: Double = wheelVector.angle.IEEErem(360.0);

        // Convert large negative angles
        if (angle <= -180.0)
            angle += 360.0

        // Convert large postive angles
        if (angle > 180.0)
            angle -= 360.0

        // Return the swerve angle in the proper hemisphere (-180 to 180 degrees)
        return angle
    }
}
