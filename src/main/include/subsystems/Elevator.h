#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include "Constants.h"

class Elevator : public frc2::SubsystemBase
{
    public:

    /// @brief Class to support the elevator subsystem.
        Elevator()
        {
            // Configure the elevator motor
            ConfigureElevatorMotor(CanConstants::ElevatorMotorCanId);
        };
        
        /// @brief Method to set the elevator height.
        /// @param position The setpoint for the elevator height.
        void SetHeight(units::length::meter_t position)
        {
            // Compute the number of turns based on the specficied position
            units::angle::turn_t newPosition = (units::angle::turn_t) (position.value() * ElevatorContants::PositionToTurnsConversionFactor);
        
            // Set the elevator set position
            m_elevatorMotor->SetControl(m_motionMagicVoltage.WithPosition(newPosition).WithSlot(0));
        };
        
    

    private:

        /// @brief Method to configure the elevator motor using MotionMagic.
        /// @param motorCanId The CAN identifier for the elevator motor.
        void ConfigureElevatorMotor(int motorCanId)
        {
            // Instantiate the elevator motor
            m_elevatorMotor = new ctre::phoenix6::hardware::TalonFX{motorCanId, CanConstants::CanBus};
        
            // Create the elevator motor configuration
            ctre::phoenix6::configs::TalonFXConfiguration elevatorMotorConfiguration{};
        
            // Add the Motor Output section settings
            ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = elevatorMotorConfiguration.MotorOutput;
            motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        
            ctre::phoenix6::configs::Slot0Configs &slot0Configs = elevatorMotorConfiguration.Slot0;
            slot0Configs.kS = ElevatorContants::S;
            slot0Configs.kV = ElevatorContants::V;
            slot0Configs.kA = ElevatorContants::A;
            slot0Configs.kP = ElevatorContants::P;
            slot0Configs.kI = ElevatorContants::I;
            slot0Configs.kD = ElevatorContants::D;
        
            // Configure gear ratio
            ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = elevatorMotorConfiguration.Feedback;
            feedbackConfigs.SensorToMechanismRatio = ElevatorContants::SensorToMechanismRatio;
        
            // Configure Motion Magic
            ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = elevatorMotorConfiguration.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorContants::MotionMagicCruiseVelocity;
            motionMagicConfigs.MotionMagicAcceleration   = ElevatorContants::MotionMagicAcceleration;
            motionMagicConfigs.MotionMagicJerk           = ElevatorContants::MotionMagicJerk;
        
            // Apply the configuration to the drive motor
            ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
            for (int attempt = 0; attempt < ChassisConstants::MotorConfigurationAttempts; attempt++)
            {
                // Apply the configuration to the drive motor
                status = m_elevatorMotor->GetConfigurator().Apply(elevatorMotorConfiguration);
        
                // Check if the configuration was successful
                if (status.IsOK())
                break;
            }
        
            // Determine if the last configuration load was successful
            if (!status.IsOK())
                std::cout << "***** ERROR: Could not configure elevator motor. Error: " << status.GetName() << std::endl;
        };
        
        
        ctre::phoenix6::hardware::TalonFX           *m_elevatorMotor;
        ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicVoltage{0_tr};
};
