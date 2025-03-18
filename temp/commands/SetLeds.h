#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Leds.h"

class SetLeds : public frc2::CommandHelper<frc2::Command, SetLeds>
{
    public:

        explicit SetLeds(int Mode,                       Leds *leds);
        explicit SetLeds(int Mode, units::second_t time, Leds *leds);

        void     Initialize()          override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;
        bool     RunsWhenDisabled()    const override;

    private:

        int             m_mode;           // The LED mode for the command
        units::second_t m_time;           // The length of time that the LEDS will be set to the given mode. Infinite by default.
        Leds           *m_leds;           // Pointer to the LED subsystem class

        bool            m_timed = false;  // Determines if the LED sequence is timed
        units::second_t m_startTime;      // The start of the LED sequence
};

/// @brief Command to set the LED mode.
/// @param Mode The LED mode.
/// @param m_leds The LED subsystem.
SetLeds::SetLeds(int Mode, Leds *leds) : m_mode(Mode), m_leds(leds)
{
    // Set the command name
    SetName("SetLeds");

    // Declare subsystem dependencies
    AddRequirements({m_leds});

}

/// @brief Command to set the LED mode.
/// @param Mode The LED mode.
/// @param m_leds The LED subsystem.
SetLeds::SetLeds(int Mode, units::second_t time, Leds *leds) : m_mode(Mode), m_time(time), m_leds(leds)
{
    // Set the command name
    SetName("SetLeds");

    // Declare subsystem dependencies
    AddRequirements({m_leds});

    // Remember the LED mode
    m_mode = Mode;

    // Indicate that the LED sequence has a time-out
    m_timed = true;
}

/// @brief Called just before this Command runs the first time.
void SetLeds::Initialize()
{
    // Set the LED mode
    m_leds->SetMode((LedMode) m_mode);

    // Get the LED sequence start time
    m_startTime = frc::GetTime();
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool SetLeds::IsFinished()
{
    // Determine if a timed LED sequence
    if (m_timed == false)
        return false;

    // Determine if the LED sequence is complete
    if (frc::GetTime() - m_startTime > m_time)
        return true;

    return false;
}

/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void SetLeds::End(bool interrupted)
{

}

/// @brief Indicates if the command runs when the robot is disabled.
/// @return True is the command should run when the robot is disabled.
bool SetLeds::RunsWhenDisabled() const
{
    // Indicate that the command should run even when the robot is disabled
    return true;
}

