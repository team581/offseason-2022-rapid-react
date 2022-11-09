// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutonomousChooser {

  private final SendableChooser<AutonomousSettings> autonomousModeChooser = new SendableChooser<>();

  public AutonomousChooser() {
    autonomousModeChooser.setDefaultOption("Do nothing", AutonomousSettings.DO_NOTHING);
    autonomousModeChooser.addOption("Red left two ball", AutonomousSettings.RED_LEFT_TWO_BALL);
    autonomousModeChooser.addOption("blue left two ball", AutonomousSettings.BLUE_LEFT_TWO_BALL);
    autonomousModeChooser.addOption("Red right two ball", AutonomousSettings.RED_RIGHT_TWO_BALL);
    autonomousModeChooser.addOption("blue right two ball", AutonomousSettings.BLUE_RIGHT_TWO_BALL);
    autonomousModeChooser.addOption("center two ball", AutonomousSettings.CENTER_TWO_BALL);
  }

  public Command getAutonomousCommand(RobotContainer robotContainer) {
    switch (autonomousModeChooser.getSelected(robotContainer)) {
      case AutonomousSettings.DO_NOTHING:
        return this.getDoNothingAuto(robotContainer);
      case AutonomousSettings.RED_LEFT_TWO_BALL:
        return this.getRedLeftTwoBall(robotContainer);
      case AutonomousSettings.BLUE_LEFT_TWO_BALL:
        return this.getBlueLeftTwoBall(robotContainer);
      case AutonomousSettings.RED_RIGHT_TWO_BALL:
        return this.getRedRightTwoBall(robotContainer);
      case AutonomousSettings.BLUE_RIGHT_TWO_BALL:
        return this.getBlueRightTwoBall(robotContainer);
      case AutonomousSettings.CENTER_TWO_BALL:
        return this.getCenterTwoBall(robotContainer);
    }
    return this.getDoNothingAuto(robotContainer);
  }

  private Command getDoNothingAuto(RobotContainer robotContainer) {
    robotContainer.imuSubsystem.setAngle(AutonomousSettings.DO_NOTHING.zeroAngle);
  }

  private Command getRedLeftTwoBall(RobotContainer robotContainer) {
    robotContainer.imuSubsystem.setAngle(AutonomousSettings.RED_LEFT_TWO_BALL.zeroAngle);
  }

  private Command getBlueLeftTwoBall(RobotContainer robotContainer) {
    robotContainer.imuSubsystem.setAngle(AutonomousSettings.BLUE_LEFT_TWO_BALL.zeroAngle);
  }

  private Command getRedRightTwoBall(RobotContainer robotContainer) {
    robotContainer.imuSubsystem.setAngle(AutonomousSettings.RED_RIGHT_TWO_BALL.zeroAngle);
  }

  private Command getBlueRightTwoBall(RobotContainer robotContainer) {
    robotContainer.imuSubsystem.setAngle(AutonomousSettings.BLUE_RIGHT_TWO_BALL.zeroAngle);
  }

  private Command getCenterTwoBall(RobotContainer robotContainer) {
    robotContainer.imuSubsystem.setAngle(AutonomousSettings.CENTER_TWO_BALL.zeroAngle);
  }
}
