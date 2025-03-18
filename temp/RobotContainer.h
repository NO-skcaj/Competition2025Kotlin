#pragma once

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>

// Subsystems
#include "subsystems/AprilTags.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Leds.h"

// Commands
#include "commands/AutonomousDoNothing.h"
#include "commands/ChassisDrive.h"
#include "commands/ChassisDriveDistance.h"
#include "commands/ChassisDriveTime.h"
#include "commands/ChassisSetFieldCentricity.h"
#include "commands/ChassisSetSwerveWheelAnglesToZero.h"
#include "commands/SetLeds.h"

#include "Constants.h"

/// @brief Class to instantiate the robot subsystems and commands along with the operator controls
class RobotContainer
{
    public:

        // Method that returns a pointer to the singleton instance of the RobotContainer class
        static RobotContainer *GetInstance();

        // Method to get a pointer to the selected autonomous command
        frc2::Command         *GetAutonomousCommand();

        // Method to set the swerve wheels to zero degrees based on the absolute encoder
        void                   SetSwerveWheelAnglesToZero();

        // Methods to get a reference to the robot joysticks
        frc::Joystick         *GetDriverController();
        frc::XboxController   *GetOperatorController();

        double                 Forward();
        double                 Strafe();
        double                 Angle();

        // Instantiate the robot subsystems
        AprilTags              m_aprilTags;
        Drivetrain             m_drivetrain;
        Elevator               m_elevator;
        Leds                   m_leds;

    private:

        // Private class constructor to configure the robot and SmartDashboard configuration
        RobotContainer();

        // Method to bind the joystick controls to the robot commands
        void   ConfigureButtonBindings();

        double GetExponentialValue(double joystickValue, double deadZone, double exponent);

        // Singleton reference to the class (returned by the GetInstance Method)
        static RobotContainer                *m_robotContainer;

        // Joysticks
        frc::Joystick                         m_driverController{ControllerConstants::DriverControllerUsbPort};
        frc::XboxController                   m_operatorController{ControllerConstants::JoystickOperatorUsbPort};

        // Autonomous command chooser
        frc::SendableChooser<frc2::Command *> m_autonomousChooser;
};


// Reference to the RobotContainer singleton class
RobotContainer *RobotContainer::m_robotContainer = NULL;

#pragma region GetInstance
/// @brief Method to return a pointer to the RobotContainer class.
/// @return Pointer to the RobotContainer class.
RobotContainer *RobotContainer::GetInstance()
{
    // Detrermine if the class has already been instantiated
    if (m_robotContainer == NULL)
    {
        // Instantiate the class
        m_robotContainer = new RobotContainer();
    }

    // Return the class pointer
    return m_robotContainer;
}
#pragma endregion

#pragma region RobotContainer
/// @brief Method to configure the robot and SmartDashboard configuration.
RobotContainer::RobotContainer()
{
    frc::SmartDashboard::PutData("ChassisDrive: Stop",       new ChassisDriveDistance(0, 0.0, &m_drivetrain));
    frc::SmartDashboard::PutData("DriveDistance: OneMeter",  new ChassisDriveDistance(1, 0.5, &m_drivetrain));
    frc::SmartDashboard::PutData("DriveDistance: TwoMeters", new ChassisDriveDistance(2, 0.5, &m_drivetrain));

    // Bind the joystick controls to the robot commands
    ConfigureButtonBindings();

    // Configure the autonomous command chooser
    m_autonomousChooser.SetDefaultOption("Do Nothing",       new AutonomousDoNothing());
    m_autonomousChooser.AddOption("Drive Forward OneMeter",  new ChassisDriveDistance(1, 0.5, &m_drivetrain));
    m_autonomousChooser.AddOption("Drive Forward TwoMeters", new ChassisDriveDistance(2, 0.5, &m_drivetrain));

    // Send the autonomous mode chooser to the SmartDashboard
    frc::SmartDashboard::PutData("Autonomous Mode", &m_autonomousChooser);

    // Set the default commands for the subsystems
    m_drivetrain.SetDefaultCommand(ChassisDrive(
        [this] { return Forward(); },
        [this] { return Strafe();  },
        [this] { return Angle();   },
        &m_drivetrain));

    m_leds.SetDefaultCommand(SetLeds(LedMode::Off, &m_leds));
}
#pragma endregion

#pragma region ConfigureButtonBindings
/// @brief Method to bind the joystick controls to the robot commands.
void RobotContainer::ConfigureButtonBindings()
{
    // Bind the driver controller buttons to the robot commands
    frc2::JoystickButton fieldCentricOn(&m_driverController, Extreme3DContants::HandleLowerLeft);
    fieldCentricOn.OnTrue(ChassisSetFieldCentricity(true, &m_drivetrain).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::JoystickButton fieldCentricOff(&m_driverController, Extreme3DContants::HandleLowerRight);
    fieldCentricOff.OnTrue(ChassisSetFieldCentricity(false, &m_drivetrain).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Bind the operator controller buttons to the robot commands
    frc2::JoystickButton setLedsOff(&m_operatorController, XBoxConstants::LeftStickButton);
    setLedsOff.OnTrue(SetLeds(LedMode::Off, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::JoystickButton setLedsStrobe(&m_operatorController, XBoxConstants::RightStickButton);
    setLedsStrobe.OnTrue(SetLeds(LedMode::Strobe, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::JoystickButton setLedsShootingAnimation{&m_operatorController, XBoxConstants::A};
    setLedsShootingAnimation.OnTrue(SetLeds(LedMode::ShootingAnimation, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::POVButton setLedsSolidGreen{&m_operatorController, XBoxConstants::Pov_0};
    setLedsSolidGreen.OnTrue(SetLeds(LedMode::SolidGreen, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::POVButton setLedsSolidRed{&m_operatorController, XBoxConstants::Pov_90};
    setLedsSolidRed.OnTrue(SetLeds(LedMode::SolidRed, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::POVButton setLedsHvaColors{&m_operatorController, XBoxConstants::Pov_180};
    setLedsHvaColors.OnTrue(SetLeds(LedMode::HvaColors, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::POVButton setLedsRainbow{&m_operatorController, XBoxConstants::Pov_270};
    setLedsRainbow.OnTrue(SetLeds(LedMode::Rainbow, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
}
#pragma endregion

#pragma region GetDriverController
/// @brief Method to return a pointer to the driver joystick.
/// @return Pointer to the driver joystick.
frc::Joystick *RobotContainer::GetDriverController()
{
    // Return the pointer to the driver joystick
    return &m_driverController;
}
#pragma endregion

#pragma region GetOperatorController
/// @brief Method to return a pointer to the controller joystick.
/// @return Pointer to the controller joystick.
frc::XboxController *RobotContainer::GetOperatorController()
{
    // Return the pointer to the operator joystick
    return &m_operatorController;
}
#pragma endregion

#pragma region GetAutonomousCommand
/// @brief Method to return a pointer to the autonomous command.
/// @return Pointer to the autonomous command
frc2::Command *RobotContainer::GetAutonomousCommand()
{
    // The selected command will be run in autonomous
    return m_autonomousChooser.GetSelected();
}
#pragma endregion

#pragma region SetSwerveWheelAnglesToZero
 /// @brief Method to set the swerve wheels to zero degrees based on the absolute encoder.
 void RobotContainer::SetSwerveWheelAnglesToZero()
 {
    // Create the command to set the swerve wheel angles to zero based on the absolute encoder
    auto command = new ChassisSetSwerveWheelAnglesToZero(&m_drivetrain);

    // Execute the command
    command->Execute();
 }
#pragma endregion

#pragma region Forward
/// @brief Method to return the forward joystick value.
/// @return The forward joystick value.
double RobotContainer::Forward()
{
    // Get the forward joystick setting
    double joystickForward = GetDriverController()->GetRawAxis(ControllerConstants::JoystickForwardIndex);

    // Use exponential function to calculate the forward value for better slow speed control
    joystickForward = GetExponentialValue(joystickForward, ControllerConstants::JoystickDeadZone, ControllerConstants::ExponentForward);

    // Return the x speed
    return -joystickForward;
}
#pragma endregion

#pragma region Strafe
/// @brief Method to return the strafe joystick value.
/// @return The strafe joystick value.
double RobotContainer::Strafe()
{
    // Get the strafe joystick setting
    double joystickStrafe = GetDriverController()->GetRawAxis(ControllerConstants::JoystickStrafeIndex);

    // Use exponential function to calculate the forward value for better slow speed control
    joystickStrafe = GetExponentialValue(joystickStrafe, ControllerConstants::JoystickDeadZone, ControllerConstants::ExponentStrafe);

    // Return the y speed
    return joystickStrafe;
}
#pragma endregion

#pragma region Angle
/// @brief Method to return the angle joystick value.
/// @return The angle joystick value.
double RobotContainer::Angle()
{
    // Get the angle joystick setting
    double joystickAngle = GetDriverController()->GetRawAxis(ControllerConstants::JoystickAngleIndex);

    // Use exponential function to calculate the forward value for better slow speed control
    if (joystickAngle)
        joystickAngle = GetExponentialValue(joystickAngle, ControllerConstants::JoystickAngleDeadZone, ControllerConstants::ExponentAngle);

    // Return the rotation speed
	return joystickAngle;
}
#pragma endregion

#pragma region GetExponentialValue
/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resultant exponential value.
double RobotContainer::GetExponentialValue(double joystickValue, double deadZone, double exponent)
{
    int    direction = 1;
    double output    = 0.0;

    // Ignore joystick input if it's too small
    if (joystickValue > -deadZone && joystickValue < deadZone)
        return 0.0;

    // Direction is either 1 or -1, based on joystick value
    if (joystickValue < 0.0)
    {
        // Reverse the direction and make the joystick value positive
        direction      = -1;
        joystickValue *= -1.0;
    }

    // Plug joystick value into exponential function
    output = direction * pow(joystickValue, exponent);

    // Ensure the range of the output
    if (output < -1.0)  output = -1.0;
    if (output >  1.0)  output =  1.0;

    // Return the calculated value
    return output;
}
#pragma endregion