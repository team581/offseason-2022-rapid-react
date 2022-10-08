package frc.robot.wrist;

public enum WristPosition {
  UP(0),
  DOWN(20);

  public final double angle;

  private WristPosition(double angle) {
    this.angle = angle;
  }
}
