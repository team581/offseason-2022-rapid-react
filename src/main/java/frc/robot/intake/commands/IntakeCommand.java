// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeSetting;
import frc.robot.intake.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private final IntakeSetting position;
  private final IntakeSubsystem intake;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intake, IntakeSetting goalPosition) {
    this.intake = intake;
    this.position = goalPosition;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intake.setPosition(this.position);
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
    return intake.atPosition(this.position);
  }
}
