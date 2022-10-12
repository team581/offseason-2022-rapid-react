package frc.robot.intake;

public enum IntakeSpeed {
  INTAKING(0.50),
  OUTTAKING(-0.40),
  STOPPED(0);

public final double percentage;

private IntakeSpeed(double percentage) {
    this.percentage = percentage;
  }
}
