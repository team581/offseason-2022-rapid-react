// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

public enum WristSetting {
  UP(-0.11, 0),
  OUTTAKING(-0.44, 0),
  INTAKING(-0.55, 0),
  STOWED(0, 0),
  HOME(0, 0.1);
  public final double angle;
  public final double voltage;

  private WristSetting(double angleSetting, double voltageSetting) {
    this.angle = angleSetting;
    this.voltage = voltageSetting;
  }
}
