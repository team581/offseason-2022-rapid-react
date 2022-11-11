// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;

public enum AutonomousSettings {
  DO_NOTHING(0),
  RED_LEFT_TWO_BALL(45),
  BLUE_LEFT_TWO_BALL(45),
  RED_RIGHT_TWO_BALL(315),
  BLUE_RIGHT_TWO_BALL(315),
  CENTER_TWO_BALL(0);
  public final Rotation2d zeroAngle;

  private AutonomousSettings(double zeroAngle) {
    this.zeroAngle = Rotation2d.fromDegrees(zeroAngle);
  }
}