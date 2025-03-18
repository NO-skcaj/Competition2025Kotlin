#include "Robot.h"

#pragma region RobotInit
/// @brief Method called when the robot class is instantiated.
void Robot::RobotInit()
{
    // Enable LiveWindow in test mode
    EnableLiveWindowInTest(true);

    // Report the robot framework usage
    HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_RobotBuilder);

    // Reset the debug message
    frc::SmartDashboard::PutString("Debug", "RobotInit");
}
#pragma endregion

#pragma region RobotPeriodic
/// @brief Method is called every robot packet, no matter the mode.
void Robot::RobotPeriodic()
{
    // Run the command scheduler
    frc2::CommandScheduler::GetInstance().Run();
}
#pragma endregion

#pragma region DisabledInit
/// @brief Method is called once each time the robot enters Disabled mode.
void Robot::DisabledInit()
{

}
#pragma endregion

#pragma region DisabledPeriodic
/// @brief Method is called periodically when the robot is disabled.
void Robot::DisabledPeriodic()
{

}
#pragma endregion

#pragma region TeleopInit
/// @brief Method is called when switching to teleoperated mode.
void Robot::TeleopInit()
{
    // Set the swerve wheels to zero
    m_container->SetSwerveWheelAnglesToZero();

    // This makes sure that the autonomous stops running when teleop starts running.
    if (m_autonomousCommand != nullptr)
    {
        // Cancel the autonomous command and set the pointer to null
        m_autonomousCommand->Cancel();
        m_autonomousCommand = nullptr;
    }
}
#pragma endregion

#pragma region TeleopPeriodic
/// @brief Method is called periodically when the robot is in tleloperated mode.
void Robot::TeleopPeriodic()
{

}
#pragma endregion

#pragma region AutonomousInit
/// @brief Method is called when switching to autonomous mode.
void Robot::AutonomousInit()
{
    // Set the swerve wheels to zero
    m_container->SetSwerveWheelAnglesToZero();

    // Get the selected autonomous command
    m_autonomousCommand = m_container->GetAutonomousCommand();

    // Determine if the chooser returned a pointer to a command
    if (m_autonomousCommand != nullptr)
    {
        // Schedule the autonomous command
        m_autonomousCommand->Schedule();
    }
}
#pragma endregion

#pragma region AutonomousPeriodic
/// @brief Method is called periodically when the robot is in autonomous mode.
void Robot::AutonomousPeriodic()
{

}
#pragma endregion

#pragma region TestInit
/// @brief Method is called when switching to test mode.
void Robot::TestInit()
{

}
#pragma endregion

#pragma region TestPeriodic
// This function is called periodically during test mode.
void Robot::TestPeriodic()
{

}
#pragma endregion

#pragma region SimulationInit
/// @brief Method is called when starting in simulation mode.
void Robot::SimulationInit()
{

}
#pragma endregion

#pragma region SimulationPeriodic
/// @brief Method is called periodically when in simulation mode.
void Robot::SimulationPeriodic()
{

}
#pragma endregion

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif