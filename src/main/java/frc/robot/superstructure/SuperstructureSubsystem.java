// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.intake.IntakeSetting;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_rollers.IntakeRollersMode;
import frc.robot.intake_rollers.IntakeRollersSubsystem;
import frc.robot.queuer.QueuerMode;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.shooter.ShooterSubsystem;

public class SuperstructureSubsystem extends SubsystemBase {
  private IntakeSubsystem intakeWrist;
  private IntakeRollersSubsystem intakeRollers;
  private QueuerSubsystem queuer;
  private ShooterSubsystem shooter;
  private RobotIntakeMode intakeMode = RobotIntakeMode.STOPPED;
  private RobotShooterMode shooterMode = RobotShooterMode.STOPPED;
  private double savedRPM = 0;
  private boolean isIntakeHoming = false;

  /** Creates a new SuperstructureSubsystem. */
  public SuperstructureSubsystem(
      IntakeSubsystem intake,
      IntakeRollersSubsystem intakeRollers,
      QueuerSubsystem queuer,
      ShooterSubsystem shooter) {
    this.intakeWrist = intake;
    this.intakeRollers = intakeRollers;
    this.queuer = queuer;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {

    // shooter logic
    if (shooterMode == RobotShooterMode.MANUAL_SPINUP) {
      this.savedRPM = 2500;
    } else if (shooterMode == RobotShooterMode.STOPPED) {
      this.savedRPM = 0;
    }
    this.shooter.setRPM(this.savedRPM);

    // intake roller logic
    if (intakeMode == RobotIntakeMode.INTAKING) {
      this.intakeRollers.setMode(IntakeRollersMode.INTAKING);
    } else if (intakeMode == RobotIntakeMode.OUTTAKING) {
      this.intakeRollers.setMode(IntakeRollersMode.OUTTAKING);
    } else if (intakeMode == RobotIntakeMode.STOPPED) {
      this.intakeRollers.setMode(IntakeRollersMode.STOPPED);
    }

    // queuer logic
    if (shooterMode == RobotShooterMode.SHOOTING) {
      this.queuer.setMode(QueuerMode.SHOOT);
    } else if (intakeMode == RobotIntakeMode.INTAKING) {
      this.queuer.setMode(QueuerMode.QUEUEING);
    } else if (intakeMode == RobotIntakeMode.OUTTAKING) {
      this.queuer.setMode(QueuerMode.EJECT);
    } else {
      this.queuer.setMode(QueuerMode.STOPPED);
    }
  }

  public void setIntakeMode(RobotIntakeMode intakeMode) {
    if(!this.isIntakeHoming) {
      this.intakeMode = intakeMode;
      if (intakeMode == RobotIntakeMode.INTAKING) {
        this.intakeWrist.setPosition(IntakeSetting.INTAKING);
      } else if (intakeMode == RobotIntakeMode.OUTTAKING) {
        this.intakeWrist.setPosition(IntakeSetting.OUTTAKING);
      }
    }
  }

  public void setIntakePosition(IntakeSetting intakeSetting) {
    if (!this.isIntakeHoming) {
      if (this.intakeMode == RobotIntakeMode.STOPPED) {
        this.intakeWrist.setPosition(intakeSetting);
      }
    }
  }

  public void setShooterMode(RobotShooterMode shooterMode) {
    this.shooterMode = shooterMode;
  }

  public void isIntakeHoming(boolean isHoming) {
    this.isIntakeHoming = isHoming;
  }

  public boolean isAtRPM() {
    return this.shooter.isAtRPM(savedRPM);
  }
}
