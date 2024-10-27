package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean cameraConnected = false;
    public double latency = 0.0;
    public double fiducialMarksID = 0.0;

    public int camerasAmount = 0;
    public int targetsCount = 0;
  }

  void updateInputs(VisionIOInputs inputs);

  String getLimelightName(int limelightNumber);

  double getLatencySeconds(int limelightNumber);

  double getTimeStampSeconds(int limelightNumber);

  boolean canSeeAprilTags(int limelightNumber);

  double getLimelightAprilTagDistance(int limelightNumber);

  int getNumberOfAprilTags(int limelightNumber);

  Pose2d getPoseFromAprilTags(int limelightNumber);

  void setHeadingInfo(double headingDegrees, double headingRateDegrees);
}
