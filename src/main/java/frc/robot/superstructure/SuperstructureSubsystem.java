// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_rollers.IntakeRollersSubsystem;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.shooter.ShooterSubsystem;

public class SuperstructureSubsystem extends SubsystemBase {
  private IntakeSubsystem intake;
  private IntakeRollersSubsystem intakeRollers;
  private QueuerSubsystem queuer;
  private ShooterSubsystem shooter;
  private RobotIntakeMode intakeMode = RobotIntakeMode.STOPPED;
  private RobotShooterMode shooterMode = RobotShooterMode.STOPPED;

  /** Creates a new SuperstructureSubsystem. */
  public SuperstructureSubsystem(
      IntakeSubsystem intake,
      IntakeRollersSubsystem intakeRollers,
      QueuerSubsystem queuer,
      ShooterSubsystem shooter) {
    this.intake = intake;
    this.intakeRollers = intakeRollers;
    this.queuer = queuer;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {}

  public void setIntakeMode(RobotIntakeMode intakeMode) {
    this.intakeMode = intakeMode;
  }

  public void setShooterMode(RobotShooterMode shooterMode) {
    this.shooterMode = shooterMode;
  }
}
