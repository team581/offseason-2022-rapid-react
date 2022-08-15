package frc.robot.misc.util.sensors;

public class SensorUnitConverter {
  public static final CancoderConverter cancoder = new CancoderConverter();
  public static final TalonFXConverter talonFX = new TalonFXConverter();

  private SensorUnitConverter() {}
}
