package frc.robot;

import com.ctre.phoenix6.hardware;

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <rev/SparkMax.h>

#include "Constants.h"

class SwerveModule()
{

    // Swerve vector struture (Drive and Angle)
    WheelVector                           m_wheelVector{0.0, 0.0};

    // Swerve drive motor
    ctre::phoenix6::hardware::TalonFX     m_driveMotor;
    ctre::phoenix6::controls::VoltageOut  m_voltageOut{0_V};  // Controller mode is VoltageOut

    // Swerve angle motor, encoder and PID controller
    rev::spark::SparkMax                  m_angleMotor;
    rev::spark::SparkClosedLoopController m_pidController;
    rev::spark::SparkRelativeEncoder      m_angleEncoder;
    ctre::phoenix6::hardware::CANcoder    m_angleAbsoluteEncoder;

    /// @brief Class constructor for the SwerveModule class.
    /// @param driveMotorCanId The CAN ID for the swerve module drive motor.
    /// @param angleMotorCanId The CAN ID for the swerve module angle motor.
    /// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
    public init(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId)
    {
        m_driveMotor(driveMotorCanId, CanConstants::CanBus),
        m_angleMotor(angleMotorCanId, rev::spark::SparkLowLevel::MotorType::kBrushless),
        m_angleAbsoluteEncoder(angleEncoderCanId, CanConstants::CanBus),
        m_angleEncoder(m_angleMotor.GetEncoder()),
        m_pidController(m_angleMotor.GetClosedLoopController())

        // Configure the drive and angle motors
        ConfigureDriveMotor();
        ConfigureAngleMotor();
    };

        /// @brief Set the swerve module angle and motor power.
        /// @param vector The wheel vector (angle and drive).
        void SetState(WheelVector vector)
        {
            // Do not change the angle if the wheel is not driven
            if (vector.Drive > 0.01 || vector.Drive < -0.01)
            {
                // Optimize the serve module vector to minimize wheel rotation on change of diretion
                OptimizeWheelAngle(vector, &m_wheelVector);
        
                // Set the angle motor PID set angle
                m_pidController.SetReference(m_wheelVector.Angle, rev::spark::SparkMax::ControlType::kPosition);
            }
            else
            {
                // Ensure the drive motor is disabled
                m_wheelVector.Drive = 0.0;
            }
        
            // Set the Drive motor voltage
            m_driveMotor.SetControl(m_voltageOut.WithOutput(m_wheelVector.Drive * 12_V));
        };
        
        /// @brief Method to read the absolute encode in Degrees.
        /// @return The absolute angle value in degrees.
        units::angle::degree_t GetAbsoluteAngle()
        {
            // The GetAbsolutePosition() method returns a value from -1 to 1
            double encoderValue = (double) m_angleAbsoluteEncoder.GetAbsolutePosition().GetValue();
        
            // To convert to degrees
            return encoderValue * 360_deg;
        };
        
        /// <summary>
        /// Method to get the swerve module wheel vector.
        /// </summary>
        /// <param name="wheelVector">Variable to return the swerve module wheel vector.</param>
        WheelVector* GetWheelVector()
        {
            // Return the wheel vector
            return &m_wheelVector;
        };
        
        /// @brief Method to get the swerve module angle encoder position.
        /// @return The angle encoder position
        double GetSwerveAngle()
        {
            // Get the angle encoder position
            return m_angleEncoder.GetPosition();
        };
        
        /// @brief Method to set the swerve wheel encoder to the forward angle.
        /// @param angle The absolute angle for the forward direction.
        void SetWheelAngleToForward(units::angle::degree_t forwardAngle)
        {
            // Set the motor angle encoder position to the forward direction
            m_angleMotor.GetEncoder().SetPosition(forwardAngle.value() - GetAbsoluteAngle().value());
        
            // Ensure the PID controller set angle is zero (forward)
            m_pidController.SetReference(0.0, rev::spark::SparkMax::ControlType::kPosition);
        };

    private:

        // Private methods

        /// @brief Method to configure the drive motor.
        void ConfigureDriveMotor()
        {
            // Create the drive motor configuration
            ctre::phoenix6::configs::TalonFXConfiguration driveMotorConfiguration{};
        
            // Add the Motor Output section settings
            ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = driveMotorConfiguration.MotorOutput;
            motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        
            // Add the Current Limits section settings
            ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = driveMotorConfiguration.CurrentLimits;
            currentLimitsConfigs.StatorCurrentLimit       = ChassisConstants::SwerveDriveMaxAmperage;
            currentLimitsConfigs.StatorCurrentLimitEnable = true;
        
            // Apply the configuration to the drive motor
            ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
            for (int attempt = 0; attempt < ChassisConstants::MotorConfigurationAttempts; attempt++)
            {
                // Apply the configuration to the drive motor
                status = m_driveMotor.GetConfigurator().Apply(driveMotorConfiguration);
        
                // Check if the configuration was successful
                if (status.IsOK())
                break;
            }
        
            // Determine if the last configuration load was successful
            if (!status.IsOK())
                std::cout << "***** ERROR: Could not configure swerve motor. Error: " << status.GetName() << std::endl;
        };
        
        /// @brief Method to configure the angle motor and encoder.
        void ConfigureAngleMotor()
        {
            // Configure the angle motor
            rev::spark::SparkBaseConfig sparkBaseConfig{};
            sparkBaseConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
            sparkBaseConfig.SecondaryCurrentLimit(ChassisConstants::SwerveAngleMaxAmperage);
            sparkBaseConfig.encoder.PositionConversionFactor(ChassisConstants::SwerveDegreesToMotorRevolutions).VelocityConversionFactor(1);
            sparkBaseConfig.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                            .Pid(SwerveConstants::P, SwerveConstants::I, SwerveConstants::D);
            m_angleMotor.Configure(sparkBaseConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
        };
        
        /// @brief Method to determine the optimal swerve module wheel angle given the desired wheel vector.
        /// @brief Note: The swerve module angle is not restricted to -180 to 180 degrees, but is the actual module angle.
        /// @param wheelVector The target swerve module wheel drive power and angle.
        void OptimizeWheelAngle(WheelVector targetWheelVector, WheelVector *wheelVector)
        {
            double driveDirection = 1.0;  // Forward direction
        
            // Convert the present wheel angle to the same hemi-sphere as the target wheel angle
            double workingAngle = ConvertAngleToTargetRange(*wheelVector);
        
            // Determine the angle between the past and desired swerve angle
            double angleDifference = targetWheelVector.Angle - workingAngle;
        
            // Determine if the angle is greater that obtuse (greater that 180 degrees)
            if (angleDifference > 180.0)
            {
                // Get the accute angle and change wheel correction direction
                angleDifference = angleDifference - 180.0;
                driveDirection *= -1.0;
            }
            else if (angleDifference < -180.0)
            {
                // Get the accute angle and change wheel correction direction
                angleDifference = angleDifference + 180.0;
                driveDirection *= -1.0;
            }
        
            // Minimize the wheel rotation
            if (angleDifference > 90.0)
            {
                // Get the minimized wheel angle and change wheel correction direction
                angleDifference = angleDifference - 180.0;
        
                // Reverse the drive direction
                driveDirection *= -1.0;
            }
            else if (angleDifference < -90.0)
            {
                // Get the minimized wheel angle and change wheel correction direction
                angleDifference = angleDifference + 180.0;
        
                // Reverse the drive direction
                driveDirection *= -1.0;
            }
        
            // Set the wheel vector to the target
            wheelVector->Angle += angleDifference;
            wheelVector->Drive  = targetWheelVector.Drive * driveDirection;
        };
        
        /// <summary>
        /// Convert any angle to the range -180 to 180 degrees.
        /// </summary>
        /// <param name="angle">The angle to convert.</param>
        /// <returns>The angle represented from -180 to 180 degrees.</returns>
        double ConvertAngleToTargetRange(WheelVector wheelVector)
        {
            // Get the angle between -360 and 360
            double angle = remainder(wheelVector.Angle, 360.0);
        
            // Convert large negative angles
            if (angle <= -180.0)
                angle += 360.0;
        
            // Convert large postive angles
            if (angle > 180.0)
                angle -= 360.0;
        
            // Return the swerve angle in the proper hemisphere (-180 to 180 degrees)
            return angle;
        };
};
