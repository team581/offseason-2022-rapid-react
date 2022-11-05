// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.superstructure.RobotIntakeMode;
import frc.robot.superstructure.SuperstructureSubsystem;

public class IntakeSubsystemCommand extends CommandBase {
  private SuperstructureSubsystem superStructure;
  private RobotIntakeMode intakeMode;

  /** Creates a new IntakeSubsystemCommand. */
  public IntakeSubsystemCommand(
      SuperstructureSubsystem superStructure, RobotIntakeMode intakeMode) {
    this.superStructure = superStructure;
    this.intakeMode = intakeMode;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.superStructure.setIntakeMode(this.intakeMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.superStructure.setIntakeMode(RobotIntakeMode.STOPPED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
