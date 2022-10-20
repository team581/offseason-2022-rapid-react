// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

public enum WristPosition {
  UP(0.25),
  OUTAKING(-0.25),
  INTAKING(-0.25);

  public final double angle;

  private WristPosition(double angle) {
    this.angle = angle;
  }
}
