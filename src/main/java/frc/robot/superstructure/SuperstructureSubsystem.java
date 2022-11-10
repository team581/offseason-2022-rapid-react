// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.intake.IntakeSetting;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_rollers.IntakeRollersMode;
import frc.robot.intake_rollers.IntakeRollersSubsystem;
import frc.robot.queuer.QueuerMode;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class SuperstructureSubsystem extends SubsystemBase {
  private static final double MAX_ROBOT_SPEED_WHILE_SHOOTING = 1;
  private final IntakeSubsystem intakeWrist;
  private final IntakeRollersSubsystem intakeRollers;
  private final QueuerSubsystem queuer;
  private final ShooterSubsystem shooter;
  private final SwerveSubsystem swerve;
  private RobotIntakeMode intakeMode = RobotIntakeMode.STOPPED;
  private RobotShooterMode shooterMode = RobotShooterMode.STOPPED;
  private double savedRPM = 0;
  private boolean isIntakeHoming = false;

  /** Creates a new SuperstructureSubsystem. */
  public SuperstructureSubsystem(
      IntakeSubsystem intake,
      IntakeRollersSubsystem intakeRollers,
      QueuerSubsystem queuer,
      ShooterSubsystem shooter,
      SwerveSubsystem swerve) {
    this.intakeWrist = intake;
    this.intakeRollers = intakeRollers;
    this.queuer = queuer;
    this.shooter = shooter;
    this.swerve = swerve;
  }

  @Override
  public void periodic() {
    // shooter logic
    if (shooterMode == RobotShooterMode.MANUAL_WARMUP) {
      this.savedRPM = 2300;
    } else if (shooterMode == RobotShooterMode.AUTO_SHOOT) {
      this.savedRPM = ShooterSubsystem.getRPMForAutoShoot();
    } else {
      this.savedRPM = 600;
    }
    this.shooter.setRPM(this.savedRPM);

    // intake roller logic
    if (intakeMode == RobotIntakeMode.INTAKING) {
      this.intakeRollers.setMode(IntakeRollersMode.INTAKING);
    } else if (intakeMode == RobotIntakeMode.OUTTAKING) {
      this.intakeRollers.setMode(IntakeRollersMode.OUTTAKING);
    } else {
      this.intakeRollers.setMode(IntakeRollersMode.STOPPED);
    }

    // queuer logic
    if (shooterMode == RobotShooterMode.AUTO_SHOOT && isAtRPM() && robotSpeedCanShoot()) {
      this.queuer.setMode(QueuerMode.SHOOT);
      this.intakeRollers.setMode(IntakeRollersMode.INTAKING);
    } else if (intakeMode == RobotIntakeMode.INTAKING) {
      this.queuer.setMode(QueuerMode.INTAKING);
    } else if (intakeMode == RobotIntakeMode.OUTTAKING) {
      this.queuer.setMode(QueuerMode.OUTTAKING);
    } else {
      this.queuer.setMode(QueuerMode.STOPPED);
    }
  }

  public void setIntakeMode(RobotIntakeMode intakeMode) {
    if (!this.isIntakeHoming) {
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
    if (!(shooterMode == RobotShooterMode.MANUAL_WARMUP
        && this.shooterMode == RobotShooterMode.AUTO_SHOOT)) {
      this.shooterMode = shooterMode;
    }
  }

  public void isIntakeHoming(boolean isHoming) {
    this.isIntakeHoming = isHoming;
  }

  public boolean isAtRPM() {
    return this.shooter.isAtRPM(savedRPM);
  }

  private boolean robotSpeedCanShoot() {
    ChassisSpeeds speed = swerve.getChassisSpeeds();
    return speed.vxMetersPerSecond < MAX_ROBOT_SPEED_WHILE_SHOOTING
        && speed.vyMetersPerSecond < MAX_ROBOT_SPEED_WHILE_SHOOTING;
  }
}
