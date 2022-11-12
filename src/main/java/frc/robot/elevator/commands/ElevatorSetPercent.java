// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.elevator.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorSetPercent extends InstantCommand {
  private final ElevatorSubsystem elevator;
  private final double percent;

  public ElevatorSetPercent(ElevatorSubsystem elevator, double percent) {
    this.elevator = elevator;
    this.percent = percent;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPercent(percent);
  }
}
