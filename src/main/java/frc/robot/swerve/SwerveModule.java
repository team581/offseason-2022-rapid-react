// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.misc.util.GearingConverter;
import frc.robot.misc.util.WheelConverter;
import frc.robot.misc.util.sensors.SensorUnitConverter;

public class SwerveModule {
  private static final SimpleMotorFeedforward DRIVE_MOTOR_FEEDFORWARD =
      new SimpleMotorFeedforward(0, 0);
  private static final SimpleMotorFeedforward STEER_MOTOR_FEEDFORWARD =
      new SimpleMotorFeedforward(0, 0);
  private static final GearingConverter DRIVE_MOTOR_GEARING_CONVERTER = new GearingConverter(10);
  private static final WheelConverter DRIVE_MOTOR_WHEEL_CONVERTER =
      WheelConverter.fromDiameter(0.1524);

  private final TalonFX driveMotor;
  private final CANCoder encoder;
  private final TalonFX steerMotor;
  private final PIDController drivePidController = new PIDController(0, 0, 0);
  private final PIDController steerPidController = new PIDController(0, 0, 0);

  public SwerveModule(TalonFX driveMotor, TalonFX steerMotor, CANCoder encoder) {
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.encoder = encoder;
  }

  public void setDesiredState(SwerveModuleState state) {}

  private double getDriveMotorVelocity() {
    final var sensorUnitsPer100msBeforeGearing = driveMotor.getSelectedSensorVelocity();
    final var sensorUnitsPer100ms =
        DRIVE_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(sensorUnitsPer100msBeforeGearing);
    final var radiansPerSecond =
        SensorUnitConverter.talonFX.sensorUnitsPer100msToRadiansPerSecond(sensorUnitsPer100ms);
    final var metersPerSecond = DRIVE_MOTOR_WHEEL_CONVERTER.radiansToDistance(radiansPerSecond);

    return metersPerSecond;
  }

  private double getSteerMotorVelocity() {
    final var sensorUnitsPer100msBeforeGearing = steerMotor.getSelectedSensorVelocity();
    final var sensorUnitsPer100ms =
        DRIVE_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(sensorUnitsPer100msBeforeGearing);
    final var radiansPerSecond =
        SensorUnitConverter.talonFX.sensorUnitsPer100msToRadiansPerSecond(sensorUnitsPer100ms);

    return radiansPerSecond;
  }

  private double getSteerMotorPosition() {
    final var degrees = encoder.getAbsolutePosition();
    final var radians = Units.degreesToRadians(degrees);

    return radians;
  }
}
