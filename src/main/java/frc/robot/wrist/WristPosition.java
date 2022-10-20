// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

public enum WristPosition {
  UP(0.25),
  OUTTAKING(-0.25),
  INTAKING(-0.25),
  STOWED(0);

  public final double angle;

  private WristPosition(double angle) {
    this.angle = angle;
  }
}
