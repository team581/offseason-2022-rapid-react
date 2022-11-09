// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.queuer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QueuerSubsystem extends SubsystemBase {
  private CANSparkMax motor;
  private DigitalInput sensor;
  private QueuerMode mode = QueuerMode.STOPPED;

  /** Creates a new QueuerSubsystem. */
  public QueuerSubsystem(CANSparkMax motor, DigitalInput sensor) {
    this.motor = motor;
    this.sensor = sensor;

    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
  }

  public boolean hasBall() {
    return sensor.get();
  }

  @Override
  public void periodic() {
    if (this.hasBall() && this.mode == QueuerMode.INTAKING) {
      this.motor.set(0);
    } else {
      this.motor.set(this.mode.percentage);
    }

    SmartDashboard.putBoolean("Queuer/Sensor", hasBall());
    SmartDashboard.putString("Queuer/Mode", this.mode.toString());
  }

  public void setMode(QueuerMode mode) {
    this.mode = mode;
  }
}
