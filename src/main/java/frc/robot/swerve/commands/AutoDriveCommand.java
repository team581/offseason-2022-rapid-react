  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.localization.Localization;
import frc.robot.swerve.SwerveSubsystem;

public class AutoDriveCommand extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Localization localization;
  private final Pose2d goal;
  private final PIDController xPid = new PIDController(0.01, 0, 0);
  private final PIDController yPid = new PIDController(0.01, 0, 0);
  private final PIDController thetaPid = new PIDController(1, 0, 0);

  /** Creates a new AutoDriveCommand. */
  public AutoDriveCommand(SwerveSubsystem swerveSubsystem, Localization localization, Pose2d goal) {
    this.swerveSubsystem = swerveSubsystem;
    this.localization = localization;
    this.goal = goal;
    addRequirements(swerveSubsystem);

    thetaPid.enableContinuousInput(-0.5, 0.5);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPid.setSetpoint(goal.getX());
    yPid.setSetpoint(goal.getY());
    thetaPid.setSetpoint(Units.radiansToRotations(goal.getRotation().getRadians()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = localization.getPose();
    double xPercentage = -xPid.calculate(currentPose.getX());
    double yPercentage = -yPid.calculate(currentPose.getY());
    double thetaPercentage =
        -thetaPid.calculate(Units.radiansToRotations(currentPose.getRotation().getRadians()));

    if (xPercentage > 0.5) {
      xPercentage = 0.5;
    } else if (xPercentage < -0.5) {
      xPercentage = -0.5;
    }

    if (yPercentage > 0.5) {
      yPercentage = 0.5;
    } else if (yPercentage < -0.5) {
      yPercentage = -0.5;
    }

    if (thetaPercentage > 1) {
      thetaPercentage = 1;
    } else if (thetaPercentage < -1) {
      thetaPercentage = -1;
    }

    swerveSubsystem.driveTeleop(xPercentage, yPercentage, thetaPercentage, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.driveTeleop(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Transform2d error = goal.minus(localization.getPose());

    return Math.abs(error.getX()) < 0.1
        && Math.abs(error.getY()) < 0.1
        && Math.abs(error.getRotation().getDegrees()) < 5;
  }
}
