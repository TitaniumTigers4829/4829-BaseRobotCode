package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.extras.vision.MegatagPoseEstimate;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import org.littletonrobotics.junction.AutoLog;

public interface VisionInterface {
  @AutoLog
  class VisionInputs {
    public boolean[] isLimelightConnected = new boolean[Limelight.values().length];

    public MegatagPoseEstimate[] limelightMegatagPoses =
        new MegatagPoseEstimate[Limelight.values().length];
    public double[] limelightLatencies = new double[Limelight.values().length];
    public int[] limelightTargets = new int[Limelight.values().length];
    public boolean[] limelightSeesAprilTags = new boolean[Limelight.values().length];

    public Pose2d[] limelightCalculatedPoses = new Pose2d[Limelight.values().length];
    public Pose2d limelightLastSeenPose = new Pose2d();
    public double[] limelightAprilTagDistances = new double[Limelight.values().length];

    public double[] limelightTimestamps = new double[Limelight.values().length];
  }

  default void updateInputs(VisionInputs inputs) {}

  default double getLatencySeconds(Limelight limelight) {
    return 0.0;
  }

  default double getTimeStampSeconds(Limelight limelight) {
    return 0.0;
  }

  default boolean canSeeAprilTags(Limelight limelight) {
    return false;
  }

  default double getLimelightAprilTagDistance(Limelight limelight) {
    return 0.0;
  }

  default int getNumberOfAprilTags(Limelight limelight) {
    return 0;
  }

  default Pose2d getPoseFromAprilTags(Limelight limelight) {
    return null;
  }

  default void setHeadingInfo(double headingDegrees, double headingRateDegrees) {}

  default Pose2d getLastSeenPose() {
    return null;
  }
}
