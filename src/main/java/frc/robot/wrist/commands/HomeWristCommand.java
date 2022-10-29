// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.WristSetting;
import frc.robot.wrist.WristSubsystem;

public class HomeWristCommand extends CommandBase {
  private final WristSubsystem wrist;


  /** Creates a new HomeWristCommand. */
  public HomeWristCommand(WristSubsystem wrist) {
    this.wrist=wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.wrist.setPosition(WristSetting.HOME);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      this.wrist.resetEncoder();
      this.wrist.setPosition(WristSetting.DONOTHING);
    }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.wrist.isHomed();
  }
}
