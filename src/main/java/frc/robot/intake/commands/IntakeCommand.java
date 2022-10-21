// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.intake.IntakeMode;
import frc.robot.intake.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCommand extends InstantCommand {
  private final IntakeSubsystem intake;
  private final IntakeMode mode;

  public IntakeCommand(IntakeSubsystem intake, IntakeMode mode) {
    this.intake = intake;
    this.mode = mode;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intake.setMode(this.mode);
  }
}
