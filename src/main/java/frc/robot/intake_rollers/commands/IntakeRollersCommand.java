// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake_rollers.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.intake_rollers.IntakeRollersMode;
import frc.robot.intake_rollers.IntakeRollersSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeRollersCommand extends InstantCommand {
  private final IntakeRollersSubsystem intakeRollers;
  private final IntakeRollersMode mode;

  public IntakeRollersCommand(IntakeRollersSubsystem intakeRollers, IntakeRollersMode mode) {
    this.intakeRollers = intakeRollers;
    this.mode = mode;
    addRequirements(intakeRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intakeRollers.setMode(this.mode);
  }
}
