// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.util.GearingConverter;
import frc.robot.misc.util.sensors.SensorUnitConverter;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX motor;
  private static GearingConverter SPROCKET_TO_CHAIN =
      GearingConverter.fromUpduction(Units.inchesToMeters(1.7507));
  private static GearingConverter GEARING = GearingConverter.fromReduction(60);
  private static double HEIGHT_TOLERANCE = 1;

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
    SmartDashboard.putNumber("Elevator Height (inch)", getHeight());
  }

  public void setPercent(double percent) {
    // Running motor at certain voltage
    motor.set(ControlMode.PercentOutput, percent);
  }

  public void setPosition(ElevatorPosition position) {
    // Move the elevator to a certain height based on encoder readings
    // motor.set(ControlMode.)
    final var heightBeforeSprocket = SPROCKET_TO_CHAIN.afterToBeforeGearing(position.height);
    final var heightBeforeGearing = GEARING.afterToBeforeGearing(heightBeforeSprocket);
    final var heightBeforeGearingSensorUnits =
        SensorUnitConverter.talonFX.rotationsToSensorUnits(heightBeforeGearing);
    motor.set(ControlMode.MotionMagic, heightBeforeGearingSensorUnits);
  }

  public double getHeight() {
    double positionSensorUnits = motor.getSelectedSensorPosition();
    double positionBeforeGearing =
        SensorUnitConverter.talonFX.sensorUnitsToRotations(positionSensorUnits);
    double positionBeforeSprocket = GEARING.beforeToAfterGearing(positionBeforeGearing);
    double elevatorHeight = SPROCKET_TO_CHAIN.beforeToAfterGearing(positionBeforeSprocket);
    return elevatorHeight;
  }

  public double getCurrent() {
    // Will return the current drawn by the elevator motor
    return motor.getStatorCurrent();
  }

  public void zeroEncoder() {
    // Set the encoder value to zero
    motor.setSelectedSensorPosition(0);
  }

  public boolean atPosition(ElevatorPosition position) {
    return Math.abs(getHeight() - position.height) < HEIGHT_TOLERANCE;
  }
}
