package frc.robot.subsystems.swerve.gyroIO;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroInterface {
  @AutoLog
  public static class GyroInputs {
    public boolean isConnected;

    public double yawDegrees;
    public Rotation2d yawDegreesRotation2d;
    public double yawVelocity;

    public Rotation2d[] odometryYawPositions;
    public double[] odometryYawTimestamps;
  }

  /**
   * Updates the inputs created in GyroInputs
   *
   * @param inputs inputs to update
   */
  void updateInputs(GyroInputs inputs);
}
