package org.team3824.frc2025.subsystems;

import kotlin.math.*

import edu.wpi.first.wpilibj2.command.SubsystemBase

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.Units.Degrees

import com.studica.frc.AHRS

import org.team3824.frc2025.CanConstants
import org.team3824.frc2025.ChassisConstants
import org.team3824.frc2025.SwerveConstants
import org.team3824.frc2025.WheelVector
import org.team3824.frc2025.SwerveModule



object Drivetrain : SubsystemBase()
{
    var m_wheelVector: Array<WheelVector> = arrayOf(WheelVector(0.0,0.0), WheelVector(0.0,0.0), WheelVector(0.0,0.0), WheelVector(0.0,0.0))

    val R: Double = sqrt((ChassisConstants.ChassisLength * ChassisConstants.ChassisLength) +
    (ChassisConstants.ChassisWidth * ChassisConstants.ChassisWidth).toDouble());

    var m_fieldCentricity: Boolean = false;                                // Field centricity flag

    val m_gyro: AHRS = AHRS(AHRS.NavXComType.kMXP_SPI);             // navX MXP using SPI

    // Create the robot swerve modules
    val m_swerveModules: Array<SwerveModule> by lazy {
        arrayOf<SwerveModule>(
            SwerveModule(
                driveMotorCanId = CanConstants.SwerveFrontRightDriveMotorCanId,
                angleMotorCanId = CanConstants.SwerveFrontRightAngleMotorCanId,
                angleEncoderCanId = CanConstants.SwerveFrontRightAngleEncoderCanId
            ),

            SwerveModule(
                driveMotorCanId = CanConstants.SwerveFrontLeftDriveMotorCanId,
                angleMotorCanId = CanConstants.SwerveFrontLeftAngleMotorCanId,
                angleEncoderCanId = CanConstants.SwerveFrontLeftAngleEncoderCanId
            ),

            SwerveModule(
                driveMotorCanId = CanConstants.SwerveRearLeftDriveMotorCanId,
                angleMotorCanId = CanConstants.SwerveRearLeftAngleMotorCanId,
                angleEncoderCanId = CanConstants.SwerveRearLeftAngleEncoderCanId
            ),

            SwerveModule(
                driveMotorCanId = CanConstants.SwerveRearRightDriveMotorCanId,
                angleMotorCanId = CanConstants.SwerveRearRightAngleMotorCanId,
                angleEncoderCanId = CanConstants.SwerveRearRightAngleEncoderCanId
            )
        )
    }

    /// @brief Class constructor for the DriveTrain subassembly.
    /// Swerve Module Indexes:
    ///
    ///          Front
    ///       +---------+ ---
    ///       |[1]   [0]|  ^       0   Front Right
    ///       |         |  |       1   Front Left
    ///       |         | Length   2   Rear Left
    ///       |         |  |       3   Rear Right
    ///       |[2]   [3]|  v
    ///       +---------+ ---
    ///       |         |
    ///       |< Width >|

    init
    {

    };

    /// @brief This method will be called once periodically.
    fun Periodic()
    {
        SmartDashboard.putNumber("Gyro Angle",        GetHeading().magnitude());
        SmartDashboard.putBoolean("Field Centricity", m_fieldCentricity);
    };

    /// @brief Method to drive the robot chassis.
    /// @param forward The forward operater input.
    /// @param strafe The strafe operater input.
    /// @param angle The angle operater input.
     fun Drive(inForward: Double, inStrafe: Double, inAngle: Double)
    {
        val forward: Double = inForward
        val strafe:  Double = inStrafe
        val angle:   Double = inAngle

        SmartDashboard.putNumber("Chassis Forward", forward);
        SmartDashboard.putNumber("Chassis Strafe",  strafe);
        SmartDashboard.putNumber("Chassis Angle",   angle);

        // Convert to field centric
        if (m_fieldCentricity)
            FieldCentricAngleConversion(forward, strafe, angle);

        // Calcualte the drive paramters
        CalculateSwerveModuleDriveAndAngle(forward, strafe, angle);

        SmartDashboard.putNumber("Front Right Drive", m_wheelVector[0].drive);
        SmartDashboard.putNumber("Front Right Angle", m_wheelVector[0].angle);
        SmartDashboard.putNumber("Front Left Drive",  m_wheelVector[1].drive);
        SmartDashboard.putNumber("Front Left Angle",  m_wheelVector[1].angle);
        SmartDashboard.putNumber("Rear Left Drive",   m_wheelVector[2].drive);
        SmartDashboard.putNumber("Rear Left Angle",   m_wheelVector[2].angle);
        SmartDashboard.putNumber("Rear Right Drive",  m_wheelVector[3].drive);
        SmartDashboard.putNumber("Rear Right Angle",  m_wheelVector[3].angle);

        // Update the swerve module
        for (swerveModuleIndex in 0..ChassisConstants.NumberOfSwerveModules)
            m_swerveModules[swerveModuleIndex].SetState(m_wheelVector[swerveModuleIndex]);

        // Read the swerve module angles and drive
        SmartDashboard.putNumber("Vector Front Right Drive", m_swerveModules[0].GetWheelVector().drive);
        SmartDashboard.putNumber("Vector Front Right Angle", m_swerveModules[0].GetWheelVector().angle);

        SmartDashboard.putNumber("Vector Front Left Drive",  m_swerveModules[1].GetWheelVector().drive);
        SmartDashboard.putNumber("Vector Front Left Angle",  m_swerveModules[1].GetWheelVector().angle);

        SmartDashboard.putNumber("Vector Rear Left Drive",   m_swerveModules[2].GetWheelVector().drive);
        SmartDashboard.putNumber("Vector Rear Left Angle",   m_swerveModules[2].GetWheelVector().angle);

        SmartDashboard.putNumber("Vector Rear Right Drive",  m_swerveModules[3].GetWheelVector().drive);
        SmartDashboard.putNumber("Vector Rear Right Angle",  m_swerveModules[3].GetWheelVector().angle);
    };

    /// @brief Method to set the robot control field centricity.
    /// @param fieldCentric Boolean to indicate if the robor control should be field centric.
     fun SetFieldCentricity(fieldCentric: Boolean)
    {
        // Set the field centric member variable
        m_fieldCentricity = fieldCentric;
    };

    /// @brief Method to set the field centricity.
    /// @return The field centricity setting.
    fun GetFieldCentricity(): Boolean
    {
        // Return the field centricity setting
        return m_fieldCentricity;
    };

    /// <summary>
    /// Method to get the specified swerve module wheel vector.
    /// </summary>
    /// <param name="swerveModuleIndex">The swerve module index.</param>
    /// <param name="wheelVector">Variable to return the specified swerve module wheel vector.</param>
    fun GetSwerveModuleWheelVector(swerveModuleIndex: Int): WheelVector
    {
        // Get the specified swerve module wheel vector
        return m_swerveModules.get(swerveModuleIndex).GetWheelVector();
    };

    /// @brief Method to set the swerve wheel to the absoulute encoder angle then zero the PID controller angle.
     fun SetWheelAnglesToZero()
    {
        // Set the swerve wheel angles to zero
        m_swerveModules.get(SwerveConstants.FrontRightIndex).SetWheelAngleToForward(SwerveConstants.FrontRightForwardAngle);
        m_swerveModules.get(SwerveConstants.FrontLeftIndex). SetWheelAngleToForward(SwerveConstants.FrontLeftForwardAngle);
        m_swerveModules.get(SwerveConstants.RearRightIndex). SetWheelAngleToForward(SwerveConstants.RearRightForwardAngle);
        m_swerveModules.get(SwerveConstants.RearLeftIndex).  SetWheelAngleToForward(SwerveConstants.RearLeftForwardAngle);
    };

    /// @brief Method to get the robot heading.
    /// @return The robot heading in degrees.
    fun GetHeading() : Angle
    {
        // Return the robot heading
        return Degrees.of(m_gyro.angle);
    };

    // Private methods

    /// <summary>
    /// Method to convert the forward and strafe into field centric values based on the gyro angle.
    ///
    /// Note: The drive motor range is 0.0 to 1.0 and the angle is in the range -180 to 180 degrees.
    /// </summary>
    /// <param name="forward">The forward power.</param>
    /// <param name="strafe">The strafe (side) power.</param>
    /// <param name="angle">The present robot angle relative to the field direction.</param>
    private fun FieldCentricAngleConversion(forwardParameter: Double, strafeParameter: Double, angle: Double) : Pair<Double,Double>
    {
        // Copy the forward and strafe method parameters
        val forward: Double = forwardParameter;
        val strafe: Double = strafeParameter;

        // Convert the angle from degrees to radians
        val radAngle = angle * PI / 180;

        // Modify the input parameters for field centric control for output
        val outForward =  forward * cos(radAngle) + strafe * sin(radAngle);
        val outStrafe  = -forward * sin(radAngle) + strafe * cos(radAngle);

        return Pair<Double, Double>(outForward, outStrafe)
    }

    /// <summary>
    /// Method to output an array of speed and rotation values for each swerve module for a drive train given
    /// the desired forward, strafe, and rotation.
    ///
    /// See: "Derivation of Inverse Kinematics for Swerve.pdf" for calcuation details located at
    ///      https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
    ///
    /// Swerve Module Indexes:
    ///
    ///          Front
    ///       +---------+ ---
    ///       |[1]   [0]|  ^       0   Front Right
    ///       |         |  |       1   Front Left
    ///       |         | Length   2   Rear Left
    ///       |         |  |       3   Rear Right
    ///       |[2]   [3]|  v
    ///       +---------+ ---
    ///       |         |
    ///       |< Width >|
    ///
    /// </summary>
    /// <param name="forward">positive value = forward movement,   negative value = backward movement</param>
    /// <param name="strafe">positive value  = right direction,    negative value = left direction</param>
    /// <param name="rotate">positive value  = clockwise rotation, negative value = counterclockwise rotation</param>
    private fun CalculateSwerveModuleDriveAndAngle(forward: Double, strafe: Double, rotate: Double)
    {
        // Create intermediate values for the speed and angle calculations
        val A: Double = strafe  - rotate * (ChassisConstants.ChassisLength / R);
        val B: Double = strafe  + rotate * (ChassisConstants.ChassisLength / R);
        val C: Double = forward - rotate * (ChassisConstants.ChassisWidth  / R);
        val D: Double = forward + rotate * (ChassisConstants.ChassisWidth  / R);

        // Calculate the wheel angle and convert radians to degrees
        m_wheelVector.get(0).angle = atan2(B, C) * 180 / PI;
        m_wheelVector.get(1).angle = atan2(B, D) * 180 / PI;
        m_wheelVector.get(2).angle = atan2(A, D) * 180 / PI;
        m_wheelVector.get(3).angle = atan2(A, C) * 180 / PI;

        // Calculate the speed
        m_wheelVector[0].drive = sqrt(B * B + C * C);
        m_wheelVector[1].drive = sqrt(B * B + D * D);
        m_wheelVector[2].drive = sqrt(A * A + D * D);
        m_wheelVector[3].drive = sqrt(A * A + C * C);

        // Normalize the speed values
        NormalizeSpeed();
    }

    /// @brief Method to normalize the Drive values for a Swerve Module.
    /// @param wheelVector Structure for returning the swerve module normalization for the drive motors.
    private fun NormalizeSpeed()
    {
        // Determine the maximum speed
        var maxSpeed: Double = m_wheelVector.get(0).drive;
        for (wheelVectorIndex in 1..ChassisConstants.NumberOfSwerveModules)
            if (m_wheelVector.get(wheelVectorIndex).drive > maxSpeed.toDouble())
                maxSpeed = m_wheelVector.get(wheelVectorIndex).drive;

        // Normalizes speeds so they're within the ranges of -1 to 1
        if (maxSpeed > 1)
            for (wheelVectorIndex in 0..ChassisConstants.NumberOfSwerveModules)
                m_wheelVector.get(wheelVectorIndex).drive /= maxSpeed;
    }
};
