// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.ElevatorSetting;
import frc.robot.elevator.ElevatorSubsystem;

public class HomeElevatorCommand extends CommandBase {
  private final ElevatorSubsystem elevator;
  /** Creates a new HomeElevatorCommand. */
  public HomeElevatorCommand(ElevatorSubsystem elevator) {
    this.elevator=elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.elevator.setPercent(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.elevator.resetEncoder();
    this.elevator.setPercent(0);
    this.elevator.setPosition(ElevatorSetting.STOWED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.elevator.isHomed();
  }
}
