// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax motor;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(CANSparkMax motor) {
    this.motor = motor;
  }

  @Override
  public void periodic() {}

  public void setSpeed(IntakeSpeed speed) {
    this.motor.set(speed.percentage);
  }
}
