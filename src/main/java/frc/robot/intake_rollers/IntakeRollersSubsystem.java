// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake_rollers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollersSubsystem extends SubsystemBase {
  private final CANSparkMax motor;
  private IntakeRollersMode mode = IntakeRollersMode.STOPPED;

  /** Creates a new IntakeRollersSubsystem. */
  public IntakeRollersSubsystem(CANSparkMax motor) {
    this.motor = motor;

    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
  }

  @Override
  public void periodic() {
    this.motor.set(this.mode.percentage);
    SmartDashboard.putString("IntakeRollers/Mode", this.mode.toString());
  }

  public void setMode(IntakeRollersMode mode) {
    this.mode = mode;
  }
}
