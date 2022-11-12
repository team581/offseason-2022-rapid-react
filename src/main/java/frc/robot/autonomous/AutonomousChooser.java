// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.intake.commands.HomeIntakeCommand;
import frc.robot.superstructure.commands.AutoAimAndShoot;
import frc.robot.swerve.commands.AutoDriveCommand;

public class AutonomousChooser {
  private final RobotContainer robotContainer;
  private final SendableChooser<AutonomousSettings> autonomousModeChooser = new SendableChooser<>();

  public AutonomousChooser(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    autonomousModeChooser.setDefaultOption("Do nothing", AutonomousSettings.DO_NOTHING);
    autonomousModeChooser.addOption("blue left two ball", AutonomousSettings.BLUE_LEFT_TWO_BALL);
    autonomousModeChooser.addOption("blue right two ball", AutonomousSettings.BLUE_RIGHT_TWO_BALL);
    autonomousModeChooser.addOption("center two ball", AutonomousSettings.CENTER_TWO_BALL);
    SmartDashboard.putData("Auto", autonomousModeChooser);
  }

  public Command getAutonomousCommand() {
    switch (autonomousModeChooser.getSelected()) {
      case DO_NOTHING:
        return this.getDoNothingAuto(robotContainer);
      case BLUE_LEFT_TWO_BALL:
        return this.getBlueLeftTwoBall(robotContainer);
      case BLUE_RIGHT_TWO_BALL:
        return this.getBlueRightTwoBall(robotContainer);
      case CENTER_TWO_BALL:
        return this.getCenterTwoBall(robotContainer);
    }
    return this.getDoNothingAuto(robotContainer);
  }

  private Command getDoNothingAuto(RobotContainer robotContainer) {
    robotContainer.localization.resetPose(new Pose2d(), AutonomousSettings.DO_NOTHING.zeroAngle);
    return new HomeIntakeCommand(robotContainer.intakeSubsystem, robotContainer.superstructure);
  }

  private Command getBlueLeftTwoBall(RobotContainer robotContainer) {
    robotContainer.localization.resetPose(
        new Pose2d(), AutonomousSettings.BLUE_LEFT_TWO_BALL.zeroAngle);
    return new HomeIntakeCommand(robotContainer.intakeSubsystem, robotContainer.superstructure)
        .andThen(new AutoDriveCommand(robotContainer.swerveSubsystem, 0, -0.35, 0, false, 3))
        .andThen(
            new AutoAimAndShoot(robotContainer.superstructure, robotContainer.driverController)
                .withTimeout(6));
  }

  private Command getBlueRightTwoBall(RobotContainer robotContainer) {
    robotContainer.localization.resetPose(
        new Pose2d(), AutonomousSettings.BLUE_RIGHT_TWO_BALL.zeroAngle);
    return new HomeIntakeCommand(robotContainer.intakeSubsystem, robotContainer.superstructure)
        .andThen(new AutoDriveCommand(robotContainer.swerveSubsystem, 0, -0.35, 0, false, 2))
        .andThen(
            new AutoAimAndShoot(robotContainer.superstructure, robotContainer.driverController)
                .withTimeout(6));
  }

  private Command getCenterTwoBall(RobotContainer robotContainer) {
    robotContainer.localization.resetPose(
        new Pose2d(), AutonomousSettings.CENTER_TWO_BALL.zeroAngle);
    return new HomeIntakeCommand(robotContainer.intakeSubsystem, robotContainer.superstructure)
        .andThen(new AutoDriveCommand(robotContainer.swerveSubsystem, 0, -0.35, 0, false, 3))
        .andThen(
            new AutoAimAndShoot(robotContainer.superstructure, robotContainer.driverController)
                .withTimeout(6));
  }
}
