// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private static final double TOLERANCE = 25;
  private static final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("limelight-upper");
  private static final NetworkTableEntry tyEntry = table.getEntry("ty");
  private static final InterpolatingTreeMap<Double, Double> distanceToRPM =
      new InterpolatingTreeMap<>();

  private static double getTY() {
    return tyEntry.getDouble(0.0);
  }

  private final CANSparkMax motor;
  private final SparkMaxPIDController pid;
  private final RelativeEncoder encoder;
  private double goalRPM = 0;
  private double voltageCompensationReference = 10;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(CANSparkMax motor) {
    this.motor = motor;
    this.pid = motor.getPIDController();
    this.encoder = motor.getEncoder();

    this.motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    this.motor.enableVoltageCompensation(voltageCompensationReference);
    this.motor.setSmartCurrentLimit(45);

    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    this.pid.setP(0.0005);
    this.pid.setI(0);
    this.pid.setD(1);
    this.pid.setIZone(0);
    this.pid.setFF(0.00023);
    this.pid.setOutputRange(0, 1);

    distanceToRPM.put(-4.78, 2650.0);
    distanceToRPM.put(-5.4, 2650.0);
    distanceToRPM.put(-20.1, 3300.0);
  }

  public double getRPM() {
    double curr = this.encoder.getVelocity();
    // shooter uses a 1:1 gearing ratio. This makes it so we don't need to convert values.
    return curr;
  }

  public static double getRPMForAutoShoot() {
    return distanceToRPM.get(getTY());
  }

  public void setRPM(double rpm) {
    this.goalRPM = rpm;
  }

  public boolean isAtRPM(double setpoint) {
    return Math.abs(setpoint - getRPM()) <= TOLERANCE;
  }

  public void shootForDistance(double ty) {
    setRPM(this.distanceToRPM.get(ty));
  }

  public boolean isAtRPMForDistance(double ty) {
    return isAtRPM(this.distanceToRPM.get(ty));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/RPM", getRPM());
    SmartDashboard.putNumber("Shooter/GoalRPM", goalRPM);
    SmartDashboard.putNumber("Shooter/Amps", this.motor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter/OutputVoltage", this.motor.getAppliedOutput());
    this.pid.setReference(goalRPM, CANSparkMax.ControlType.kVelocity);
  }
}
