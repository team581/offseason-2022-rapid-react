// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controller.DriveController;
import frc.robot.swerve.SwerveSubsystem;

public class TeleopDriveCommand extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final DriveController controller;
  private PIDController autoAimPD;

  /** Creates a new TeleopDriveCommand. */
  public TeleopDriveCommand(SwerveSubsystem swerveSubsystem, DriveController controller) {
    this.swerveSubsystem = swerveSubsystem;
    this.controller = controller;
    this.autoAimPD = new PIDController(0.01, 0, 0.1);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }

    final var slowMode = controller.leftTrigger.get();
    final var robotRelative = controller.rightTrigger.get();
    // TODO: Update this keybind based off the controls slideshow
    final var autoAim = controller.rightBumper.get();


    var sidewaysPercentage = controller.getSidewaysPercentage();
    var forwardPercentage = controller.getForwardPercentage();
    var thetaPercentage = controller.getThetaPercentage();

    if (slowMode) {
      sidewaysPercentage *= 0.5;
      forwardPercentage *= 0.5;
      thetaPercentage *= 0.5;
    }

    if (autoAim) {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-upper");
      double targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0);
      thetaPercentage = -autoAimPD.calculate(targetOffsetAngle_Horizontal, 0);
      SmartDashboard.putNumber("Pickle", thetaPercentage);
    }

    swerveSubsystem.driveTeleop(
        sidewaysPercentage, -forwardPercentage, thetaPercentage, !robotRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command never stops unless it's interrupted when another command is using the drivetrain
    return false;
  }
}
