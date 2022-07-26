// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.imu;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ImuSubsystem extends SubsystemBase {
  private final Pigeon2 imu;
  /** Creates a new ImuSubsystem. */
  public ImuSubsystem(Pigeon2 imu) {
    this.imu = imu;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro sensor", this.getRobotHeading().getDegrees());
  }

  public Rotation2d getRobotHeading() {
    return Rotation2d.fromDegrees(imu.getYaw());
  }

  public void zero() {
    this.imu.setYaw(0);
  }

  public void setAngle(double zeroAngle) {
    this.imu.setYaw(zeroAngle);
  }
}
