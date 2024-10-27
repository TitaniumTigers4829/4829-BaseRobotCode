package frc.robot.subsystems.swerve.gyroIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface GyroInterface {
  @AutoLog
  public static class GyroInputs {
    public boolean isConnected = false;
    public Rotation2d yawDegreesRotation2d = new Rotation2d();
    public Angle yawDegrees = Degrees.zero();
    public double pitchDegrees = 0.0;
    public double rollDegrees = 0.0;
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double[] accel = new double[] {0, 0, 0};
    public AngularVelocity yawVelocity = DegreesPerSecond.zero();
    public double[] odometryYawTimestamps = new double[] {};
  }

  default void updateInputs(GyroInputs inputs) {}

  default void reset() {}

  default void addOffset(Rotation2d offset) {}
}
