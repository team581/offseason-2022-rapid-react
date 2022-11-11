// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeSetting;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.superstructure.SuperstructureSubsystem;

public class HomeIntakeCommand extends CommandBase {
  private final IntakeSubsystem intake;
  private SuperstructureSubsystem superStructure;

  /** Creates a new HomeIntakeCommand. */
  public HomeIntakeCommand(IntakeSubsystem intake, SuperstructureSubsystem superStructure) {
    this.intake = intake;
    this.superStructure = superStructure;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.superStructure.isIntakeHoming(true);
    this.intake.setPosition(IntakeSetting.HOME);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intake.resetEncoder();
    this.intake.setPosition(IntakeSetting.DONOTHING);
    this.superStructure.isIntakeHoming(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.intake.isHomed();
  }
}
