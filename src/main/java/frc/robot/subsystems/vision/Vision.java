package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO visionIO;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public Vision(VisionIO visionIO) {
    // Initializing Fields
    this.visionIO = visionIO;
  }

  @Override
  public void periodic() {
    // Updates limelight inputs
    visionIO.updateInputs(inputs);
    Logger.processInputs(visionIO.getLimelightName(0), inputs);
  }

  // Add methods to support DriveCommand
  public int getNumberOfAprilTags(int limelightNumber) {
    return visionIO.getNumberOfAprilTags(limelightNumber);
  }

  public double getLimelightAprilTagDistance(int limelightNumber) {
    return visionIO.getLimelightAprilTagDistance(limelightNumber);
  }

  public double getTimeStampSeconds(int limelightNumber) {
    return visionIO.getTimeStampSeconds(limelightNumber);
  }

  public double getLatencySeconds(int limelightNumber) {
    return visionIO.getLatencySeconds(limelightNumber);
  }

  public void setHeadingInfo(double headingDegrees, double headingRateDegrees) {
    visionIO.setHeadingInfo(headingDegrees, headingRateDegrees);
  }

  @AutoLogOutput(key = "Vision/Has Targets")
  public boolean canSeeAprilTags(int limelightNumber) {
    return visionIO.canSeeAprilTags(
        limelightNumber); // Assuming we're checking the shooter limelight
  }

  public Pose2d getPoseFromAprilTags(int limelightNumber) {
    return visionIO.getPoseFromAprilTags(limelightNumber);
  }
}
