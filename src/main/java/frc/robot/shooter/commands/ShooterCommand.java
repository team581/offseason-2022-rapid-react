// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shooter.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  private ShooterSubsystem shooter;
  private double goalRPM;

  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem shooter, double goalRPM ) {
    this.shooter = shooter;
    this.goalRPM = goalRPM;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooter.setRPM(this.goalRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isAtRPM(this.goalRPM);
  }
}
