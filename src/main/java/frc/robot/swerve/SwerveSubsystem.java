// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  private static final double MAX_VELOCITY = 4.5;
  private static final double MAX_ANGULAR_VELOCITY = 20;

  private final ImuSubsystem imu;
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule rearRight;
  private final SwerveModule rearLeft;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(
      ImuSubsystem imu,
      DriveController controller,
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule rearLeft,
      SwerveModule rearRight) {
    this.imu = imu;
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearLeft = rearLeft;
    this.rearRight = rearRight;
    setDefaultCommand(new TeleopDriveCommand(this, controller));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public ChassisSpeeds getChassisSpeeds() {
    final var frontLeftState = frontLeft.getState();
    final var frontRightState = frontRight.getState();
    final var rearLeftState = rearLeft.getState();
    final var rearRightState = rearRight.getState();
    return KINEMATICS.toChassisSpeeds(
        frontLeftState, frontRightState, rearLeftState, rearRightState);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
    };
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    final var moduleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    rearLeft.setDesiredState(moduleStates[2]);
    rearRight.setDesiredState(moduleStates[3]);
  }

  public void driveTeleop(
      double sidewaysPercentage,
      double forwardPercentage,
      double thetaPercentage,
      boolean fieldRelative) {

    Translation2d robotTranslation =
        new Translation2d(sidewaysPercentage, forwardPercentage).times(MAX_VELOCITY);
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            robotTranslation.getX(),
            robotTranslation.getY(),
            thetaPercentage * MAX_ANGULAR_VELOCITY,
            fieldRelative ? imu.getRobotHeading() : new Rotation2d());
    SwerveModuleState[] moduleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY);
    setChassisSpeeds(KINEMATICS.toChassisSpeeds(moduleStates));
  }
}
