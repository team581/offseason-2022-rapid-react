// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.util.GearingConverter;

public class WristSubsystem extends SubsystemBase {
  private static final double TOLERANCE_ANGLE = 0.01;
  private static final double HOMED_CURRENT = 15;
  private static final GearingConverter GEARING = GearingConverter.fromReduction(64);
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pid;
  private WristSetting goal = WristSetting.DONOTHING;

  /** Creates a new WristSubsystem. */
  public WristSubsystem(CANSparkMax motor) {
    this.motor = motor;
    this.pid = motor.getPIDController();
    this.encoder = motor.getEncoder();

    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    this.pid.setP(0.4);
    this.pid.setI(0);
    this.pid.setD(0);
    this.pid.setIZone(0);
    this.pid.setFF(0);
    this.pid.setOutputRange(-0.75, 1);
  }

  public double getAngle() {
    double angleBeforeGearing = this.encoder.getPosition();
    double angle = GEARING.beforeToAfterGearing(angleBeforeGearing);
    return angle;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist/Position", getAngle());
    SmartDashboard.putNumber("Wrist/Amps", this.motor.getOutputCurrent());
    SmartDashboard.putNumber("Wrist/OutputVoltage", this.motor.getAppliedOutput());
    SmartDashboard.putNumber("Wrist/CommandedVoltage", goal.voltage);
    SmartDashboard.putNumber("Wrist/CommandedPosition", goal.angle);
    if (goal == WristSetting.HOME || goal == WristSetting.DONOTHING) {
      SmartDashboard.putString("Wrist/State", "Voltage");
      this.pid.setReference(goal.voltage, CANSparkMax.ControlType.kVoltage);
    } else {
      SmartDashboard.putString("Wrist/State", "Position");
      this.pid.setReference(
          WristSubsystem.GEARING.afterToBeforeGearing(goal.angle),
          CANSparkMax.ControlType.kPosition);
    }
  }

  public void setPosition(WristSetting position) {
    this.goal = position;
  }

  public boolean atPosition(WristSetting position) {
    double error = getAngleError(position);

    return Math.abs(error) < WristSubsystem.TOLERANCE_ANGLE;
  }

  /** Returns difference between given position and actual position */
  private double getAngleError(WristSetting position) {
    double angleBeforeGearing = this.encoder.getPosition();
    double angle = WristSubsystem.GEARING.beforeToAfterGearing(angleBeforeGearing);

    return position.angle - angle;
  }

  public boolean isHomed() {
    return this.motor.getOutputCurrent() > HOMED_CURRENT;
  }

  public void resetEncoder() {
    this.encoder.setPosition(0);
  }
}
