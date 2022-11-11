// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class Localization extends SubsystemBase {
  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;
  private final SwerveDriveOdometry odometry;

  /** Creates a new Localization. */
  public Localization(SwerveSubsystem swerve, ImuSubsystem imu) {
    this.swerve = swerve;
    this.imu = imu;

    final var initialHeading = imu.getRobotHeading();

    odometry = new SwerveDriveOdometry(SwerveSubsystem.KINEMATICS, initialHeading);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final var heading = imu.getRobotHeading();
    final var moduleStates = swerve.getModuleStates();

    odometry.update(heading, moduleStates);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose, Rotation2d gyroAngle) {
    odometry.resetPosition(pose, gyroAngle);
  }
}
