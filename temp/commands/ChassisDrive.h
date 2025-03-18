#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

class ChassisDrive : public frc2::CommandHelper<frc2::Command, ChassisDrive>
{
    public:

        explicit ChassisDrive(std::function<double()> forward, std::function<double()> strafe,
                              std::function<double()> angle, Drivetrain *drivetrain);

        void     Execute() override;

        void     SetFieldCentricity(bool fieldCentric);

    private:

        std::function<double()> m_forward;     // The forward speed
        std::function<double()> m_strafe;      // The strafe speed
        std::function<double()> m_angle;       // The angle speed
        Drivetrain             *m_drivetrain;  // The drivetrain subsystem;
};

/// @brief Command to support the driver chassis drive command.
/// @param forward The forward driver input.
/// @param strafe The strafe driver input.
/// @param angle The angle driver input.
/// @param m_drivetrain The drive train subsystem.
ChassisDrive::ChassisDrive(std::function<double()> forward,
                           std::function<double()> strafe,
                           std::function<double()> angle,
                           Drivetrain *drivetrain) :
                           m_forward{std::move(forward)},
                           m_strafe{std::move(strafe)},
                           m_angle{std::move(angle)},
                           m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDrive");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}


/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDrive::Execute()
{
    // Perform the chassis drive
    m_drivetrain->Drive(m_forward(), m_strafe(), m_angle());
}

