// TigerLib 2024

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extras.interpolators.MultiLinearInterpolator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;

public abstract class DriveCommandBase extends Command {

  private final MultiLinearInterpolator oneAprilTagLookupTable =
      new MultiLinearInterpolator(VisionConstants.ONE_APRIL_TAG_LOOKUP_TABLE);
  private final MultiLinearInterpolator twoAprilTagLookupTable =
      new MultiLinearInterpolator(VisionConstants.TWO_APRIL_TAG_LOOKUP_TABLE);

  private final Vision vision;
  private final SwerveDrive swerveDrive;

  private double lastTimeStampSeconds = 0;

  /**
   * An abstract class that handles pose estimation while driving.
   *
   * @param driveSubsystem The subsystem for the swerve drive
   * @param vision The subsystem for vision measurements
   */
  public DriveCommandBase(SwerveDrive swerveDrive, Vision vision) {
    this.swerveDrive = swerveDrive;
    this.vision = vision;
    // It is important that you do addRequirements(driveSubsystem, vision) in whatever
    // command extends this
  }

  @Override
  public void execute() {
    swerveDrive.addPoseEstimatorSwerveMeasurement();
    vision.setHeadingInfo(
        swerveDrive.getPose().getRotation().getDegrees(), swerveDrive.getGyroRate());
    calculatePoseFromLimelight(VisionConstants.SHOOTER_LIMELIGHT_NUMBER);
    calculatePoseFromLimelight(VisionConstants.FRONT_LEFT_LIMELIGHT_NUMBER);
    calculatePoseFromLimelight(VisionConstants.FRONT_RIGHT_LIMELIGHT_NUMBER);
  }

  public void calculatePoseFromLimelight(int limelightNumber) {
    double currentTimeStampSeconds = lastTimeStampSeconds;

    // Updates the robot's odometry with april tags
    if (vision.canSeeAprilTags(limelightNumber)) {
      currentTimeStampSeconds = vision.getTimeStampSeconds(limelightNumber);

      double distanceFromClosestAprilTag = vision.getLimelightAprilTagDistance(limelightNumber);

      if (vision.getNumberOfAprilTags(limelightNumber) == 1) {
        double[] standardDeviations =
            oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        swerveDrive.setPoseEstimatorVisionConfidence(
            standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      } else if (vision.getNumberOfAprilTags(limelightNumber) > 1) {
        double[] standardDeviations =
            twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        swerveDrive.setPoseEstimatorVisionConfidence(
            standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      }

      swerveDrive.addPoseEstimatorVisionMeasurement(
          vision.getPoseFromAprilTags(limelightNumber),
          Timer.getFPGATimestamp() - vision.getLatencySeconds(limelightNumber));
    }

    lastTimeStampSeconds = currentTimeStampSeconds;
  }
}
