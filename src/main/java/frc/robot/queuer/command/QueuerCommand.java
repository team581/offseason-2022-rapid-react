// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.queuer.command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.queuer.QueuerMode;
import frc.robot.queuer.QueuerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class QueuerCommand extends InstantCommand {
  private QueuerSubsystem queuer;
  private QueuerMode mode;

  public QueuerCommand(QueuerSubsystem queuer, QueuerMode mode) {
    this.queuer = queuer;
    this.mode = mode;
    addRequirements(queuer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.queuer.setMode(this.mode);
  }
}
