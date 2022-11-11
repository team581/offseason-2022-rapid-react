// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controller.DriveController;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.commands.TeleopDriveCommand;

public class SwerveSubsystem extends SubsystemBase {

  private static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.381, 0.381);
  private static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.381, -0.381);
  private static final Translation2d REAR_LEFT_LOCATION = new Translation2d(-0.381, 0.381);
  private static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(-0.381, -0.381);
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, REAR_LEFT_LOCATION, REAR_RIGHT_LOCATION);
  public static final double MAX_VELOCITY = 4.5;
  private static final double MAX_ANGULAR_VELOCITY = 20;

  private final ImuSubsystem imu;
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backRight;
  private final SwerveModule backLeft;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(
      ImuSubsystem imu,
      DriveController controller,
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule backLeft,
      SwerveModule backRight) {
    this.imu = imu;
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;
    setDefaultCommand(new TeleopDriveCommand(this, controller));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    this.frontLeft.logValues();
    this.frontRight.logValues();
    this.backLeft.logValues();
    this.backRight.logValues();
  }

  public ChassisSpeeds getChassisSpeeds() {
    final var frontLeftState = frontLeft.getState();
    final var frontRightState = frontRight.getState();
    final var rearLeftState = backLeft.getState();
    final var rearRightState = backRight.getState();
    return KINEMATICS.toChassisSpeeds(
        frontLeftState, frontRightState, rearLeftState, rearRightState);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
    };
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean openLoop) {
    final var moduleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    frontLeft.setDesiredState(moduleStates[0], openLoop);
    frontRight.setDesiredState(moduleStates[1], openLoop);
    backLeft.setDesiredState(moduleStates[2], openLoop);
    backRight.setDesiredState(moduleStates[3], openLoop);
  }

  public void driveTeleop(
      double sidewaysPercentage,
      double forwardPercentage,
      double thetaPercentage,
      boolean fieldRelative) {

    SmartDashboard.putNumber("Sideways percentage", sidewaysPercentage);
    SmartDashboard.putNumber("Forward percentage", forwardPercentage);
    SmartDashboard.putNumber("Theta percentage", thetaPercentage);

    Translation2d robotTranslation =
        new Translation2d(forwardPercentage, -1 * sidewaysPercentage).times(MAX_VELOCITY);
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            robotTranslation.getX(),
            robotTranslation.getY(),
            -1 * thetaPercentage * MAX_ANGULAR_VELOCITY,
            fieldRelative ? imu.getRobotHeading() : new Rotation2d());
    SwerveModuleState[] moduleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY);
    setChassisSpeeds(KINEMATICS.toChassisSpeeds(moduleStates), true);
  }

  public double getAngle() {
    return imu.getRobotHeading().getDegrees();
  }
}
