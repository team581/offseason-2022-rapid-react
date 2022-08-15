// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  private final WPI_TalonFX driveMotor;
  private final CANCoder encoder;
  private final WPI_TalonFX steerMotor;
  private final PIDController drivePidController = new PIDController(0, 0, 0);
  private final ProfiledPIDController steerPidController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX steerMotor, CANCoder encoder) {
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.encoder = encoder;
  }

  public void setDesiredState(SwerveModuleState state) {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorVelocity = getDriveMotorVelocity();

    state = SwerveModuleState.optimize(state, steerMotorPosition);

    final var steerFeedbackVoltage =
        steerPidController.calculate(steerMotorPosition.getRadians(), state.angle.getRadians());
    final var steerFeedforwardVoltage =
        STEER_MOTOR_FEEDFORWARD.calculate(steerPidController.getSetpoint().velocity);
    final var steerVoltage = steerFeedbackVoltage + steerFeedforwardVoltage;
    steerMotor.setVoltage(steerVoltage);

    final var driveFeedbackVoltage =
        drivePidController.calculate(driveMotorVelocity, state.speedMetersPerSecond);
    final var driveFeedforwardkVoltage =
        DRIVE_MOTOR_FEEDFORWARD.calculate(state.speedMetersPerSecond);
    final var driveVoltage = driveFeedbackVoltage + driveFeedforwardkVoltage;
    driveMotor.setVoltage(driveVoltage);
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

  private Rotation2d getSteerMotorPosition() {
    final var degrees = encoder.getAbsolutePosition();

    return Rotation2d.fromDegrees(degrees);
  }
}
