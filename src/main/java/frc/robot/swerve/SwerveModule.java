// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.misc.util.CtreModuleState;
import frc.robot.misc.util.GearingConverter;
import frc.robot.misc.util.WheelConverter;
import frc.robot.misc.util.sensors.SensorUnitConverter;

public class SwerveModule {
  private static final SimpleMotorFeedforward DRIVE_MOTOR_FEEDFORWARD =
      new SimpleMotorFeedforward(0, 0);
  private static final double DRIVE_MOTOR_MAX_VOLTAGE = 12;
  private static final GearingConverter DRIVE_MOTOR_GEARING_CONVERTER = new GearingConverter(10);
  private static final GearingConverter STEER_MOTOR_GEARING_CONVERTER = new GearingConverter(10);
  private static final WheelConverter DRIVE_MOTOR_WHEEL_CONVERTER =
      WheelConverter.fromDiameter(0.1524);

  private final TalonFX driveMotor;
  private final CANCoder encoder;
  private final TalonFX steerMotor;

  public SwerveModule(TalonFX driveMotor, TalonFX steerMotor, CANCoder encoder) {
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.encoder = encoder;

    driveMotor.config_kP(0, 0);
    driveMotor.config_kI(0, 0);
    driveMotor.config_kD(0, 0);
    driveMotor.config_kF(0, 0);

    steerMotor.config_kP(0, 0);
    steerMotor.config_kI(0, 0);
    steerMotor.config_kD(0, 0);
    steerMotor.config_kF(0, 0);
  }

  public void setDesiredState(SwerveModuleState state) {
    final var steerMotorPosition = getSteerMotorPosition();

    state = CtreModuleState.optimize(state, steerMotorPosition);

    final var radians = (state.angle.getRadians());
    final var radiansBeforeGearing = STEER_MOTOR_GEARING_CONVERTER.afterToBeforeGearing(radians);
    final var sensorUnits = SensorUnitConverter.talonFX.radiansToSensorUnits(radiansBeforeGearing);
    steerMotor.set(ControlMode.Position, sensorUnits);

    final var metersPerSecond = state.speedMetersPerSecond;
    final var radiansPerSecond = DRIVE_MOTOR_WHEEL_CONVERTER.distanceToRadians(metersPerSecond);
    final var sensorUnitsPer100ms =
        SensorUnitConverter.talonFX.radiansPerSecondToSensorUnitsPer100ms(radiansPerSecond);
    final var sensorUnitsPer100msBeforeGearing =
        DRIVE_MOTOR_GEARING_CONVERTER.afterToBeforeGearing(sensorUnitsPer100ms);
    driveMotor.set(
        ControlMode.Velocity,
        sensorUnitsPer100msBeforeGearing,
        DemandType.ArbitraryFeedForward,
        DRIVE_MOTOR_FEEDFORWARD.kv / DRIVE_MOTOR_MAX_VOLTAGE);
  }

  public SwerveModuleState getState() {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorVelocity = getDriveMotorVelocity();

    return new SwerveModuleState(driveMotorVelocity, steerMotorPosition);
  }

  private Rotation2d getSteerMotorPosition() {
    final var degrees = encoder.getAbsolutePosition();

    return Rotation2d.fromDegrees(degrees);
  }

  private double getDriveMotorVelocity() {
    final var sensorUnitsPer100msBeforeGearing = driveMotor.getSelectedSensorVelocity();
    final var sensorUnitsPer100ms =
        DRIVE_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(sensorUnitsPer100msBeforeGearing);
    final var radiansPerSecond =
        SensorUnitConverter.talonFX.sensorUnitsPer100msToRadiansPerSecond(sensorUnitsPer100ms);
    final var metersPerSecond = DRIVE_MOTOR_WHEEL_CONVERTER.radiansToDistance(radiansPerSecond);

    return metersPerSecond;
  }
}
