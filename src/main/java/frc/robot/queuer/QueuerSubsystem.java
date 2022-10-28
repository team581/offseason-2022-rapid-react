// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.queuer;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QueuerSubsystem extends SubsystemBase {
  private CANSparkMax motor;
  private DigitalInput sensor;
  private QueuerMode mode;

  /** Creates a new QueuerSubsystem. */
  public QueuerSubsystem(CANSparkMax motor, DigitalInput sensor) {
    this.motor = motor;
    this.sensor = sensor;
  }

  public boolean hasBall() {
    return sensor.get();
  }

  @Override
  public void periodic() {
    this.motor.set(this.mode.percentage);
  }

  public void setMode(QueuerMode mode) {
    this.mode = mode;
  }
}
