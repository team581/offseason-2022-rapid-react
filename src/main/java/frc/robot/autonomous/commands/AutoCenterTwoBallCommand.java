// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.superstructure.commands.AutoAimAndShoot;
import frc.robot.swerve.commands.AutoDriveCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCenterTwoBallCommand extends SequentialCommandGroup {
  /** Creates a new AutoCenterTwoBallCommand. */
  public AutoCenterTwoBallCommand(RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // TODO(jonah): inline this
    addRequirements(robotContainer.swerveSubsystem);
    addCommands(
        new AutoDriveCommand(
            robotContainer.swerveSubsystem,
            robotContainer.localization,
            new Pose2d(-6, 0, new Rotation2d())),
        new AutoAimAndShoot(robotContainer.superstructure, robotContainer.driverController));
  }
}
