// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.queuer;

public enum QueuerMode {
  STOPPED(0),
  INTAKING(0.10),
  OUTTAKING(-0.5),
  SHOOT(0.25);

  public final double percentage;

  private QueuerMode(double percentage) {
    this.percentage = percentage;
  }
}
