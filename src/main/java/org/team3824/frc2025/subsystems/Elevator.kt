package org.team3824.frc2025.subsystems;

import com.ctre.phoenix6.configs.*
import edu.wpi.first.wpilibj2.command.SubsystemBase

import edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.MotionMagicVoltage

import com.ctre.phoenix6.hardware.TalonFX

import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import org.team3824.frc2025.CanConstants
import org.team3824.frc2025.ElevatorConstants

object Elevator : SubsystemBase()
{

    private val m_elevatorMotor:      TalonFX by lazy  { TalonFX(CanConstants.ElevatorMotorCanId) } // delays instantiation until init/config
    private val m_motionMagicVoltage: MotionMagicVoltage = MotionMagicVoltage(Rotations.of(0.0));


    /// @brief Class to support the elevator subsystem.
    init
    {

        // Configure the elevator motor
        configureElevatorMotor();
    };

    /// @brief Method to set the elevator height.
    /// @param position The setpoint for the elevator height.
    fun setHeight(position: Distance)
    {
        // Compute the number of turns based on the specified position
        val newPosition: Angle = Rotations.of(position.magnitude() * ElevatorConstants.PositionToTurnsConversionFactor);

        // Set the elevator set position
        m_elevatorMotor.setControl(m_motionMagicVoltage.withPosition(newPosition).withSlot(0));
    };

    /// @brief Method to configure the elevator motor using MotionMagic.
    /// @param motorCanId The CAN identifier for the elevator motor.
    private fun configureElevatorMotor()
    {
        // Create the elevator motor configuration
        val elevatorMotorConfiguration = TalonFXConfiguration();

        // Add the Motor Output section settings
        val motorOutputConfigs: MotorOutputConfigs = elevatorMotorConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        val slot0Configs: Slot0Configs = elevatorMotorConfiguration.Slot0;
        slot0Configs.kS = ElevatorConstants.S;
        slot0Configs.kV = ElevatorConstants.V;
        slot0Configs.kA = ElevatorConstants.A;
        slot0Configs.kP = ElevatorConstants.P;
        slot0Configs.kI = ElevatorConstants.I;
        slot0Configs.kD = ElevatorConstants.D;

        // Configure gear ratio
        val feedbackConfigs: FeedbackConfigs = elevatorMotorConfiguration.Feedback;
        feedbackConfigs.SensorToMechanismRatio = ElevatorConstants.SensorToMechanismRatio;

        // Configure Motion Magic
        val motionMagicConfigs: MotionMagicConfigs = elevatorMotorConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration   = ElevatorConstants.MotionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk           = ElevatorConstants.MotionMagicJerk;

        // Apply the configuration to the drive motor
        for (attempt in  1..ElevatorConstants.MotorConfigurationAttempts)
        {
            val status = m_elevatorMotor.configurator.apply(elevatorMotorConfiguration);
            if (status.isOK) break
            else if (!m_elevatorMotor.configurator.apply(elevatorMotorConfiguration).isOK) throw IllegalStateException("Elevator Config Failed");
        }
    };
};
