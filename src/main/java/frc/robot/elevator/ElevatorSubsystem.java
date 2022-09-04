// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.util.GearingConverter;
import frc.robot.misc.util.sensors.SensorUnitConverter;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX motor;
  private static GearingConverter SPROCKET_TO_CHAIN = GearingConverter.fromUpduction(Units.inchesToMeters(1.7507));
  private static GearingConverter GEARING = GearingConverter.fromReduction(60);





  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(TalonFX motor) {
    this.motor = motor;
    motor.config_kF(0, 0);
    motor.config_kP(0, 0);
    motor.config_kI(0, 0);
    motor.config_kD(0, 0);
    motor.configMotionAcceleration(0);
    motor.configMotionCruiseVelocity(0);
    motor.configClosedloopRamp(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercent(double percent) {
    // Running motor at certain voltage
    motor.set(ControlMode.PercentOutput, percent);
  }

  public void setHeight(double height) {
    // Move the elevator to a certain height based on encoder readings
    // motor.set(ControlMode.)
    final var heightBeforeSprocket = SPROCKET_TO_CHAIN.afterToBeforeGearing(height);
    final var heightBeforeGearing = GEARING.afterToBeforeGearing(heightBeforeSprocket);
    final var heightBeforeGearingSensorUnits = SensorUnitConverter.talonFX.rotationsToSensorUnits(heightBeforeGearing);
    motor.set(ControlMode.MotionMagic, heightBeforeGearingSensorUnits);


  }

  public double getCurrent() {
    // Will return the current drawn by the elevator motor
    return motor.getStatorCurrent();
  }

  public void zeroEncoder() {
    // Set the encoder value to zero
  }
}
