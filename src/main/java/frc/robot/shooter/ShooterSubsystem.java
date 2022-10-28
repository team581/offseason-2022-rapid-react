// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private static final double TOLERANCE = 50;
  private final CANSparkMax motor;
  private final SparkMaxPIDController pid;
  private final RelativeEncoder encoder;
  private double goalRPM = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(CANSparkMax motor) {
    this.motor = motor;
    this.pid = motor.getPIDController();
    this.encoder = motor.getEncoder();

    this.pid.setP(0);
    this.pid.setI(0);
    this.pid.setD(0);
    this.pid.setIZone(0);
    this.pid.setFF(0);
    this.pid.setOutputRange(0, 0.5);
  }

  public double getRPM() {
    double curr = this.encoder.getVelocity();
    // shooter uses a 1:1 gearing ratio. This makes it so we don't need to convert values.
    return curr;
  }

  public void setRPM(double rpm) {
    this.goalRPM = rpm;
  }

  public boolean isAtRPM(double setpoint) {
    return Math.abs(setpoint - getRPM()) <= TOLERANCE;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/RPM", getRPM());
    this.pid.setReference(goalRPM, CANSparkMax.ControlType.kVelocity);
  }
}
