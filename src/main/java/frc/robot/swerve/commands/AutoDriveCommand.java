// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swerve.SwerveSubsystem;

public class AutoDriveCommand extends CommandBase {
  private final double sidewaysPercentage;
  private final double forwardPercentage;
  private final double thetaPercentage;
  private final SwerveSubsystem swerveSubsystem;
  private final boolean fieldRelative;
  private final double duration;
  private final Timer timer = new Timer();

  /** Creates a new AutoDriveCommand. */
  public AutoDriveCommand(
      SwerveSubsystem swerveSubsystem,
      double sidewaysPercentage,
      double forwardPercentage,
      double thetaPercentage,
      boolean fieldRelative,
      double duration) {
    this.swerveSubsystem = swerveSubsystem;
    this.sidewaysPercentage = sidewaysPercentage;
    this.forwardPercentage = forwardPercentage;
    this.thetaPercentage = thetaPercentage;
    this.fieldRelative = fieldRelative;
    this.duration = duration;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.driveTeleop(
        sidewaysPercentage, -forwardPercentage, thetaPercentage, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}
