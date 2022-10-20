// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.intake.commands.IntakeCommand;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax motor;
  private IntakeMode mode = IntakeMode.STOPPED;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(CANSparkMax motor) {
    this.motor = motor;


  }

  @Override
  public void periodic() {
    this.motor.set(this.mode.percentage);
  }

  public void setMode(IntakeMode mode) {
    this.mode = mode;
  }
}
