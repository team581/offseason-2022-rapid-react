// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final Rotation2d angleOffset;
  public final SwerveCorner corner;

  public SwerveModuleConstants(Rotation2d angleOffset, SwerveCorner corner) {
    this.angleOffset = angleOffset;
    this.corner = corner;
  }
}
