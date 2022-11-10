// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swerve.SwerveSubsystem;

public class AutoAimAndShoot extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController autoAimPD = new PIDController(0.01, 0, 0.001);

  /** Creates a new AutoAimAndShoot. */
  public AutoAimAndShoot(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-upper");
    double targetOffsetAngleHorizontal = table.getEntry("tx").getDouble(0);
    double thetaPercentage = -autoAimPD.calculate(targetOffsetAngleHorizontal, 0);
    if (Math.abs(thetaPercentage) < 0.02) thetaPercentage = 0;

    swerveSubsystem.driveTeleop(0, 0, thetaPercentage, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
