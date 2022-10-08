// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.util.GearingConverter;

public class WristSubsystem extends SubsystemBase {
  private static GearingConverter GEARING = GearingConverter.fromReduction(60);
  private final CANSparkMax motor;
  private final SparkMaxPIDController pid;
  private final RelativeEncoder encoder;

  /** Creates a new WristSubsystem. */
  public WristSubsystem(CANSparkMax motor) {
    this.motor = motor;
    this.pid = motor.getPIDController();
    this.encoder = motor.getEncoder();
  }

  public double getAngle() {
    double angleBeforeGearing = this.encoder.getPosition();
    double angle = GEARING.beforeToAfterGearing(angleBeforeGearing);
    return angle;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
