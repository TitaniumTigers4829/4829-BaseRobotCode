// TigerLib 2023

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.MultiLinearInterpolator;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public abstract class DriveCommandBase extends Command {

  private final MultiLinearInterpolator oneAprilTagLookupTable = 
    new MultiLinearInterpolator(VisionConstants.ONE_APRIL_TAG_LOOKUP_TABLE);
  private final MultiLinearInterpolator twoAprilTagLookupTable = 
    new MultiLinearInterpolator(VisionConstants.TWO_APRIL_TAG_LOOKUP_TABLE);

  private final PoseEstimationSubsystem poseEstimator;
  private final VisionSubsystem visionSubsystem;
  private final DriveSubsystem driveSubsystem;

  private double lastTimeStampSeconds = 0;
  private int ticksAfterSeeing = 0;

  /**
   * An abstract class that handles pose estimation while driving.
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   */
  public DriveCommandBase(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = new DriveSubsystem();
    this.poseEstimator = new PoseEstimationSubsystem(driveSubsystem);
  }

  @Override
  public void execute() {
    // Updates the pose estimator using the swerve modules
    poseEstimator.addPoseEstimatorSwerveMeasurement();

    // Updates the robot's odometry with april tags
    double currentTimeStampSeconds = lastTimeStampSeconds;

    if (visionSubsystem.canSeeAprilTags()) {
      ticksAfterSeeing++;
      currentTimeStampSeconds = visionSubsystem.getTimeStampSeconds();

      double distanceFromClosestAprilTag = visionSubsystem.getDistanceFromClosestAprilTag();
      // Sets the pose estimator confidence in vision based off of number of april tags and distance
      if (visionSubsystem.getNumberOfAprilTags() == 1) {
        double[] standardDeviations = oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        poseEstimator.setPoseEstimatorVisionConfidence(standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      } else if (visionSubsystem.getNumberOfAprilTags() > 1) {
        double[] standardDeviations = twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        poseEstimator.setPoseEstimatorVisionConfidence(standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      }

      // Only updates the pose estimator if the limelight pose is new and reliable
      if (currentTimeStampSeconds > lastTimeStampSeconds && ticksAfterSeeing > VisionConstants.FRAMES_BEFORE_ADDING_VISION_MEASUREMENT) {
        Pose2d limelightVisionMeasurement = visionSubsystem.getPoseFromAprilTags();
        poseEstimator.addPoseEstimatorVisionMeasurement(limelightVisionMeasurement, Timer.getFPGATimestamp() - visionSubsystem.getLatencySeconds());
      }
    } else {
      ticksAfterSeeing = 0;
    }

    lastTimeStampSeconds = currentTimeStampSeconds;
  }

}