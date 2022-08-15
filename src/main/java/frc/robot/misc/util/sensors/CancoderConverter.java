package frc.robot.misc.util.sensors;

public class CancoderConverter extends SensorUnitConverterBase {
  public CancoderConverter() {
    super(4096);
  }

  public int radiansPerSecondToSensorUnitsPer100ms(double radiansPerSecond) {
    return (int) Math.round(radiansToSensorUnits(radiansPerSecond) / 10);
  }

  public double sensorUnitsPer100msToRadiansPerSecond(double sensorUnitsPer100ms) {
    return sensorUnitsToRadians(sensorUnitsPer100ms * 10);
  }
}
