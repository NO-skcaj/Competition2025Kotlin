package org.team3824.frc2025.commands

import edu.wpi.first.wpilibj2.command.Command

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance

import org.team3824.frc2025.subsystems.Elevator

class ElevatorSetHeight(private val m_height: Distance,
                        private val m_elevator: Elevator) : Command()
{
    init
    {
        addRequirements(m_elevator)
    }

        override fun execute()
        {
            m_elevator.setHeight(m_height);
        }

        override fun isFinished(): Boolean
        {
            return true
        }

};