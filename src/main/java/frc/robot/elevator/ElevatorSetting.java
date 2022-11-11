// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

public enum ElevatorSetting {
  STOWED(0),
  DEPLOYED(20),
  LATCHED(10);

  public final double height;

  private ElevatorSetting(double height) {
    this.height = height;
  }
}
