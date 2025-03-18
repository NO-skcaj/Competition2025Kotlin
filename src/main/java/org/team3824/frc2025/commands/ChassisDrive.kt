package org.team3824.frc2025.commands

import edu.wpi.first.wpilibj2.command.Command

import org.team3824.frc2025.subsystems.Drivetrain

class ChassisDrive(private var m_forward: () -> Double,     // The forward speed
                   private var m_strafe:  () -> Double,     // The strafe speed
                   private var m_angle:   () -> Double,     // The angle speed
                   private var m_drivetrain: Drivetrain) : Command()     // The drivetrain subsystem)
{



    init {
        addRequirements(m_drivetrain)
    }

    override fun execute()
    {
        m_drivetrain.Drive(m_forward(), m_strafe(), m_angle())
    }
}