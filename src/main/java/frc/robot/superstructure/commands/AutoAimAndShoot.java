// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controller.DriveController;
import frc.robot.superstructure.RobotShooterMode;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class AutoAimAndShoot extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController autoAimPD = new PIDController(0.01, 0, 0.001);
  private final SuperstructureSubsystem superStructure;

  private final DriveController controller;

  /** Creates a new AutoAimAndShoot. */
  public AutoAimAndShoot(SuperstructureSubsystem superStructure, DriveController controller) {
    this.swerveSubsystem = superStructure.swerve;
    this.superStructure = superStructure;
    this.controller = controller;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.superStructure.setShooterMode(RobotShooterMode.AUTO_SHOOT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.superStructure.setShooterMode(RobotShooterMode.AUTO_SHOOT);

    double forwardPercentage = 0;
    double sidewaysPercentage = 0;

    if (DriverStation.isTeleopEnabled()) {
      forwardPercentage = -controller.getForwardPercentage();
      sidewaysPercentage = controller.getSidewaysPercentage();
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-upper");
    double targetOffsetAngleHorizontal = table.getEntry("tx").getDouble(0);
    double thetaPercentage = -autoAimPD.calculate(targetOffsetAngleHorizontal, 0);
    if (Math.abs(thetaPercentage) < 0.02) thetaPercentage = 0;

    swerveSubsystem.driveTeleop(sidewaysPercentage, forwardPercentage, thetaPercentage, true);
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
