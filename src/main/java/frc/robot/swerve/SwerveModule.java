// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private static final SimpleMotorFeedforward DRIVE_MOTOR_FEEDFORWARD =
      new SimpleMotorFeedforward(0, 0);
  private static final SimpleMotorFeedforward STEER_MOTOR_FEEDFORWARD =
      new SimpleMotorFeedforward(0, 0);

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
}
