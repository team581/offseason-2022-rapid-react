// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.superstructure.RobotShooterMode;
import frc.robot.superstructure.SuperstructureSubsystem;

public class ManualShooterCommand extends CommandBase {
  private SuperstructureSubsystem superStructure;

  /** Creates a new ManualShooterCommand. */
  public ManualShooterCommand(SuperstructureSubsystem superStructure) {
    this.superStructure = superStructure;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.superStructure.setShooterMode(RobotShooterMode.MANUAL_SHOOT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.superStructure.setShooterMode(RobotShooterMode.MANUAL_SHOOT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.superStructure.setShooterMode(RobotShooterMode.STOPPED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
