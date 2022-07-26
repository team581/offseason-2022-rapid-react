// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.AutonomousChooser;
import frc.robot.controller.ButtonController;
import frc.robot.controller.DriveController;
import frc.robot.elevator.ElevatorSetting;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.commands.ElevatorGoToPosition;
import frc.robot.elevator.commands.ElevatorSetPercent;
import frc.robot.example.ExampleSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSetting;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.commands.HomeIntakeCommand;
import frc.robot.intake_rollers.IntakeRollersSubsystem;
import frc.robot.localization.Localization;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.superstructure.RobotIntakeMode;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.commands.AutoAimAndShoot;
import frc.robot.superstructure.commands.IntakeSubsystemCommand;
import frc.robot.superstructure.commands.ManualShooterCommand;
import frc.robot.swerve.SwerveCorner;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveModuleConstants;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.commands.TeleopDriveCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveController driverController =
      new DriveController(Constants.DRIVER_CONTROLLER_PORT);
  private final ButtonController operatorController =
      new ButtonController(Constants.OPERATOR_CONTROLLER_PORT);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final ImuSubsystem imuSubsystem = new ImuSubsystem(new Pigeon2(1));
  private final IntakeRollersSubsystem intakeRollersSubsystem =
      new IntakeRollersSubsystem(new CANSparkMax(15, MotorType.kBrushless));
  public final IntakeSubsystem intakeSubsystem =
      new IntakeSubsystem(new CANSparkMax(16, MotorType.kBrushless));
  private final QueuerSubsystem queuerSubsystem =
      new QueuerSubsystem(new CANSparkMax(17, MotorType.kBrushless), new DigitalInput(0));
  private final ShooterSubsystem shooterSubsystem =
      new ShooterSubsystem(new CANSparkMax(18, MotorType.kBrushless));
  public final SwerveSubsystem swerveSubsystem =
      new SwerveSubsystem(
          imuSubsystem,
          driverController,
          new SwerveModule(
              new SwerveModuleConstants(
                  Rotation2d.fromDegrees(104.6), SwerveCorner.FRONT_LEFT, false, false),
              new TalonFX(2),
              new TalonFX(3),
              new CANCoder(10)),
          new SwerveModule(
              new SwerveModuleConstants(
                  Rotation2d.fromDegrees(78.95), SwerveCorner.FRONT_RIGHT, false, false),
              new TalonFX(4),
              new TalonFX(5),
              new CANCoder(11)),
          new SwerveModule(
              new SwerveModuleConstants(
                  Rotation2d.fromDegrees(-148), SwerveCorner.BACK_LEFT, false, false),
              new TalonFX(6),
              new TalonFX(7),
              new CANCoder(12)),
          new SwerveModule(
              new SwerveModuleConstants(
                  Rotation2d.fromDegrees(-62.53), SwerveCorner.BACK_RIGHT, false, false),
              new TalonFX(8),
              new TalonFX(9),
              new CANCoder(13)));
  public final Localization localization = new Localization(swerveSubsystem, imuSubsystem);
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(new TalonFX(14));
  public final SuperstructureSubsystem superstructure =
      new SuperstructureSubsystem(
          intakeSubsystem,
          intakeRollersSubsystem,
          queuerSubsystem,
          shooterSubsystem,
          swerveSubsystem);
  public final AutonomousChooser autonomousChooser = new AutonomousChooser(this);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.swerveSubsystem.setDefaultCommand(
        new TeleopDriveCommand(this.swerveSubsystem, this.driverController));
    this.elevatorSubsystem.setDefaultCommand(
        new ElevatorSetPercent(this.elevatorSubsystem, 0)
            .perpetually()
            .withName("PerpetualElevatorSetPercent"));
    // TODO: Try doing sequentual(ShooterCommand, parallel(QueuerCommand.perpetually(),
    // ShooterCommand.perpetually()))
    // this.shooterSubsystem.setDefaultCommand(
    // new ShooterCommand(this.shooterSubsystem, 0).perpetually());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // driver controls
    driverController
        .rightBumper
        .whenPressed(new ElevatorGoToPosition(elevatorSubsystem, ElevatorSetting.DEPLOYED))
        .whenReleased(new ElevatorGoToPosition(elevatorSubsystem, ElevatorSetting.LATCHED));

    this.driverController.startButton.whenPressed(() -> this.imuSubsystem.zero());
    driverController.leftTrigger.whileActiveContinuous(
        new IntakeSubsystemCommand(superstructure, RobotIntakeMode.INTAKING));
    driverController.leftBumper.whileActiveContinuous(
        new IntakeSubsystemCommand(superstructure, RobotIntakeMode.OUTTAKING));
    driverController.rightTrigger.whileActiveContinuous(
        new AutoAimAndShoot(superstructure, driverController));

    // operator controls
    // new Trigger(() -> operatorController.getRightY() > 0.5)
    //     .whileActiveContinuous(new ElevatorSetPercent(elevatorSubsystem, 0.15));
    // new Trigger(() -> operatorController.getRightY() < -0.5)
    //     .whileActiveContinuous(new ElevatorSetPercent(elevatorSubsystem, -0.15));
    operatorController.backButton.whenPressed(
        new HomeIntakeCommand(this.intakeSubsystem, superstructure));
    operatorController.leftTrigger.whileActiveContinuous(
        () -> this.superstructure.setIntakePosition(IntakeSetting.INTAKING));
    operatorController.leftBumper.whileActiveContinuous(
        () -> this.superstructure.setIntakePosition(IntakeSetting.UP));
    operatorController.rightTrigger.whileActiveContinuous(new ManualShooterCommand(superstructure));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousChooser.getAutonomousCommand();
  }
}
