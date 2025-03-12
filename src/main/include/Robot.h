#pragma once

#include <hal/FRCUsageReporting.h>
#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>

#include "RobotContainer.h"

#include "Constants.h"

class Robot : public frc::TimedRobot
{
    public:

        /// @brief Method called when the robot class is instantiated.
        void RobotInit()
        {
            // Enable LiveWindow in test mode
            EnableLiveWindowInTest(true);
        
            // Report the robot framework usage
            HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_RobotBuilder);
        
            // Reset the debug message
            frc::SmartDashboard::PutString("Debug", "RobotInit");
        };
        
        /// @brief Method is called every robot packet, no matter the mode.
        void RobotPeriodic()
        {
            // Run the command scheduler
            frc2::CommandScheduler::GetInstance().Run();
        };
        
        /// @brief Method is called once each time the robot enters Disabled mode.
        void DisabledInit()
        {
        
        };
        
        /// @brief Method is called periodically when the robot is disabled.
        void DisabledPeriodic()
        {
        
        };
        
        /// @brief Method is called when switching to teleoperated mode.
        void TeleopInit()
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
        };
        
        /// @brief Method is called periodically when the robot is in tleloperated mode.
        void TeleopPeriodic()
        {
        
        };
        
        /// @brief Method is called when switching to autonomous mode.
        void AutonomousInit()
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
        };
        
        /// @brief Method is called periodically when the robot is in autonomous mode.
        void AutonomousPeriodic()
        {
        
        };
        
        /// @brief Method is called when switching to test mode.
        void TestInit()
        {
        
        };
        
        // This function is called periodically during test mode.
        void TestPeriodic()
        {
        
        };
        
        /// @brief Method is called when starting in simulation mode.
        void SimulationInit()
        {
        
        };
        
        
        /// @brief Method is called periodically when in simulation mode.
        void SimulationPeriodic()
        {
        
        };
        
        
    private:

        // Pointer to the autonomous command
        frc2::Command  *m_autonomousCommand = nullptr;

        // Instantiate the Robot container and get a pointer to the class
        RobotContainer *m_container         = RobotContainer::GetInstance();
};
