// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controller.ButtonController;
import frc.robot.controller.DriveController;
import frc.robot.controller.LogitechF310DirectInputController;
import frc.robot.example.ExampleSubsystem;
import frc.robot.example.commands.ExampleCommand;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveController driverController =
      new DriveController(new XboxController(Constants.DRIVER_CONTROLLER_PORT));
  private final ButtonController copilotController =
      new ButtonController(
          new LogitechF310DirectInputController(Constants.COPILOT_CONTROLLER_PORT));
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ImuSubsystem imuSubsystem = new ImuSubsystem(new Pigeon2(1));
  private final SwerveSubsystem swerveSubsystem =
      new SwerveSubsystem(
          imuSubsystem,
          driverController,
          new SwerveModule(new TalonFX(2), new TalonFX(3), new CANCoder(10)),
          new SwerveModule(new TalonFX(4), new TalonFX(5), new CANCoder(11)),
          new SwerveModule(new TalonFX(6), new TalonFX(7), new CANCoder(12)),
          new SwerveModule(new TalonFX(8), new TalonFX(9), new CANCoder(13)));

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
