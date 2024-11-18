package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionInterface visionInterface;
  private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

  public Vision(VisionInterface visionInterface) {
    // Initializing Fields
    this.visionInterface = visionInterface;
  }

  @Override
  public void periodic() {
    // Updates limelight inputs
    visionInterface.updateInputs(inputs);
    Logger.processInputs(visionInterface.getLimelightName(0), inputs);
  }

  // Add methods to support DriveCommand
  public int getNumberOfAprilTags(int limelightNumber) {
    return visionInterface.getNumberOfAprilTags(limelightNumber);
  }

  public double getLimelightAprilTagDistance(int limelightNumber) {
    return visionInterface.getLimelightAprilTagDistance(limelightNumber);
  }

  public double getTimeStampSeconds(int limelightNumber) {
    return visionInterface.getTimeStampSeconds(limelightNumber);
  }

  public double getLatencySeconds(int limelightNumber) {
    return visionInterface.getLatencySeconds(limelightNumber);
  }

  public void setHeadingInfo(double headingDegrees, double headingRateDegrees) {
    visionInterface.setHeadingInfo(headingDegrees, headingRateDegrees);
  }

  @AutoLogOutput(key = "Vision/Has Targets")
  public boolean canSeeAprilTags(int limelightNumber) {
    return visionInterface.canSeeAprilTags(
        limelightNumber); // Assuming we're checking the shooter limelight
  }

  public Pose2d getPoseFromAprilTags(int limelightNumber) {
    return visionInterface.getPoseFromAprilTags(limelightNumber);
  }
}
