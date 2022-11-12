// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.ElevatorSetting;
import frc.robot.elevator.ElevatorSubsystem;

public class ElevatorGoToPosition extends CommandBase {
  private final ElevatorSubsystem elevator;
  private final ElevatorSetting position;

  /** Creates a new ElevatorGoToPosition. */
  public ElevatorGoToPosition(ElevatorSubsystem elevator, ElevatorSetting position) {
    this.elevator = elevator;
    this.position = position;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPosition(position);
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
    return elevator.atPosition(position);
  }
}
