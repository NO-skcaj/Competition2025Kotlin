#include "commands/AutonomousCommand.h"

AutonomousCommand::AutonomousCommand()
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("AutonomousCommand");
}

// Called just before this Command runs the first time
void AutonomousCommand::Initialize()
{

}

// Called repeatedly when this Command is scheduled to run
void AutonomousCommand::Execute()
{

}

// Make this return true when this Command no longer needs to run execute()
bool AutonomousCommand::IsFinished()
{
    return false;
}

// Called once after isFinished returns true
void AutonomousCommand::End(bool interrupted)
{

}

bool AutonomousCommand::RunsWhenDisabled() const
{
    return false;
}
