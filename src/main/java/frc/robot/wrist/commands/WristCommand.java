// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.WristPosition;
import frc.robot.wrist.WristSubsystem;

public class WristCommand extends CommandBase {
  private final WristPosition position;
  private final WristSubsystem wrist;

  /** Creates a new WristCommand. */
  public WristCommand(WristSubsystem wrist, WristPosition goalPosition) {
    this.wrist = wrist;
    this.position = goalPosition;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.wrist.setPosition(this.position);
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
    return wrist.atPosition(this.position);
  }
}
