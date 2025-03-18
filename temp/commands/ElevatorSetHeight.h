#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Elevator.h"

class ElevatorSetHeight : public frc2::CommandHelper<frc2::Command, ElevatorSetHeight>
{
    public:
      
        explicit ElevatorSetHeight(units::length::meter_t height, Elevator *elevator);

        void     Execute()    override;
        bool     IsFinished() override;
        
    private:

        units::length::meter_t m_height;
        Elevator              *m_elevator;
};

ElevatorSetHeight::ElevatorSetHeight(units::length::meter_t height, Elevator *elevator) : m_height(height), m_elevator(elevator)
{
    // Set the command name
    SetName("Elevator");

    // Declare subsystem dependencies
    AddRequirements(elevator);
}

// Called repeatedly when this Command is scheduled to run
void ElevatorSetHeight::Execute()
{
    // Set the elevator height setpoint
    m_elevator->SetHeight(m_height);
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ElevatorSetHeight::IsFinished()
{
    // Execute only runs once
    return true;
}


