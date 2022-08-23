// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controller.DriveController;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.commands.TeleopDriveCommand;

public class SwerveSubsystem extends SubsystemBase {

  private static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.381, 0.381);
  private static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.381, -0.381);
  private static final Translation2d REAR_LEFT_LOCATION = new Translation2d(-0.381, 0.381);
  private static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(-0.381, -0.381);
  private static final SwerveDriveKinematics KINEMATICS =
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
      SwerveModule rearRight,
      SwerveModule rearLeft) {
    this.imu = imu;
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearRight = rearRight;
    this.rearLeft = rearLeft;
    setDefaultCommand(new TeleopDriveCommand(this, controller));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    final var chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardPercentage * MAX_VELOCITY,
            -sidewaysPercentage * MAX_VELOCITY,
            thetaPercentage * MAX_ANGULAR_VELOCITY,
            fieldRelative ? imu.getRobotHeading() : new Rotation2d());
    setChassisSpeeds(chassisSpeeds);
  }
}
