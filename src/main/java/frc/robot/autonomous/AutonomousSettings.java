// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;

public enum AutonomousSettings {
  DO_NOTHING(0),
  BLUE_LEFT_TWO_BALL(328),
  BLUE_RIGHT_TWO_BALL(80),
  CENTER_TWO_BALL(0),
  BLUE_LEFT_THREE_BALL(328);
  public final Rotation2d zeroAngle;

  private AutonomousSettings(double zeroAngle) {
    this.zeroAngle = Rotation2d.fromDegrees(zeroAngle);
  }
}
