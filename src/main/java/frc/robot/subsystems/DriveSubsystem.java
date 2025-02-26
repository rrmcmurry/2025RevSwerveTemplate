// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {
  // Create 4 instances of SwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // Create NavX AHRS Gyroscope
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);


   /**
   * Method to drive the robot using joystick info.
   *
   * @param forward       Speed of the robot in the forward, forward positive.
   * @param strafe        Speed of the robot in the sideways direction, left negative.
   * @param rotation      Rate of the robot rotation, left negative.
   * @param fieldRelative Whether the provided speeds are relative to the field.   
   */
  public void drive(double forward, double strafe, double rotation, boolean fieldRelative) {
    
    // Convert the commanded speeds into the correct units for the drivetrain, make strafe and rotation left positive numbers as expected for swerve
    double forwardDelivered = forward * DriveConstants.kMaxSpeedMetersPerSecond;
    double strafeDelivered = -strafe * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = -rotation * DriveConstants.kMaxAngularSpeed;
    double currentangle = -m_gyro.getAngle() % 360;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardDelivered, strafeDelivered, rotDelivered, Rotation2d.fromDegrees(currentangle))
            : new ChassisSpeeds(forwardDelivered, strafeDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  
  }

}
