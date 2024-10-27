package frc.robot.subsystems.swerve.gyroIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroInterface {
  @AutoLog
  public static class GyroInputs {
    public boolean isConnected = false;
    public Rotation2d yawDegreesRotation2d = new Rotation2d();
    public double yawDegrees = 0.0;
    public double pitchDegrees = 0.0;
    public double rollDegrees = 0.0;
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double[] accel = new double[] {0, 0, 0};
    public double yawVelocity = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
  }

  default void updateInputs(GyroInputs inputs) {}

  default void reset() {}

  default void addOffset(Rotation2d offset) {}
}
