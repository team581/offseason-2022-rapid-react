// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

public enum IntakeSetting {
  UP(-0.11, 0),
  OUTTAKING(-0.40, 0),
  INTAKING(-0.55, 0),
  STOWED(-0.05, 0),
  HOME(0, 2),
  DONOTHING(0, 0);
  public final double angle;
  public final double voltage;

  private IntakeSetting(double angleSetting, double voltageSetting) {
    this.angle = angleSetting;
    this.voltage = voltageSetting;
  }
}
