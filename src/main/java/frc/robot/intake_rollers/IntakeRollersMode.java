// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake_rollers;

public enum IntakeRollersMode {
  INTAKING(-0.6),
  OUTTAKING(0.6),
  STOPPED(0);

  public final double percentage;

  private IntakeRollersMode(double percentage) {
    this.percentage = percentage;
  }
}
