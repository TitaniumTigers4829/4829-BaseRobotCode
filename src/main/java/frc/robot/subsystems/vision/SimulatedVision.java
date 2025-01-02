package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.extras.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

// Simulate the vision system.
// Please see the following link for example code
// https://github.com/PhotonVision/photonvision/blob/2a6fa1b6ac81f239c59d724da5339f608897c510/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java
public class SimulatedVision extends PhysicalVision {
  PhotonCameraSim shooterCameraSim;
  PhotonCameraSim frontLeftCameraSim;
  PhotonCameraSim frontRightCameraSim;
  private final VisionSystemSim visionSim;
  private final Supplier<Pose2d> robotSimulationPose;

  private final int kResWidth = 1280;
  private final int kResHeight = 800;

  private int[] tagCount = new int[Limelight.values().length];
  private double[] apriltagDist = new double[Limelight.values().length];

  public SimulatedVision(Supplier<Pose2d> robotSimulationPose) {
    super();
    this.robotSimulationPose = robotSimulationPose;
    // Create the vision system simulation which handles cameras and targets on the
    // field.
    visionSim = new VisionSystemSim("main");

    // Add all the AprilTags inside the tag layout as visible targets to this
    // simulated field.
    visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
    // visionSim.addVisionTargets(TargetModel.kAprilTag36h11);

    // Create simulated camera properties. These can be set to mimic your actual
    // camera.
    var cameraProperties = new SimCameraProperties();
    // cameraProperties.setCalibration(kResWidth, kResHeight, Rotation2d.fromDegrees(97.7));
    // cameraProperties.setCalibError(0.35, 0.10);
    cameraProperties.setFPS(15);
    cameraProperties.setAvgLatencyMs(20);
    cameraProperties.setLatencyStdDevMs(5);

    // Create a PhotonCameraSim which will update the linked PhotonCamera's values
    // with visible targets.
    // Instance variables
    shooterCameraSim =
        new PhotonCameraSim(getSimulationCamera(Limelight.SHOOTER), cameraProperties);
    frontLeftCameraSim =
        new PhotonCameraSim(getSimulationCamera(Limelight.FRONT_LEFT), cameraProperties);
    frontRightCameraSim =
        new PhotonCameraSim(getSimulationCamera(Limelight.FRONT_RIGHT), cameraProperties);

    visionSim.addCamera(shooterCameraSim, VisionConstants.SHOOTER_TRANSFORM);
    visionSim.addCamera(frontLeftCameraSim, VisionConstants.FRONT_LEFT_TRANSFORM);
    visionSim.addCamera(frontRightCameraSim, VisionConstants.FRONT_RIGHT_TRANSFORM);

    // Enable the raw and processed streams. (http://localhost:1181 / 1182)
    shooterCameraSim.enableRawStream(true);
    shooterCameraSim.enableProcessedStream(true);
    frontLeftCameraSim.enableRawStream(true);
    frontLeftCameraSim.enableProcessedStream(true);
    frontRightCameraSim.enableRawStream(true);
    frontRightCameraSim.enableProcessedStream(true);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    shooterCameraSim.enableDrawWireframe(true);
    frontLeftCameraSim.enableDrawWireframe(true);
    frontRightCameraSim.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    // Abuse the updateInputs periodic call to update the sim

    // Move the vision sim robot on the field
    if (robotSimulationPose.get() != null) {
      visionSim.update(robotSimulationPose.get());
      Logger.recordOutput("Vision/SimIO/updateSimPose", robotSimulationPose.get());
    }

    for (Limelight limelight : Limelight.values()) {
      writeToTable(
          getSimulationCamera(limelight).getAllUnreadResults(),
          getLimelightTable(limelight),
          limelight);
      inputs.limelightTargets[limelight.getId()] = getNumberOfAprilTags(limelight);
      inputs.limelightAprilTagDistances[limelight.getId()] =
          getLimelightAprilTagDistance(limelight);
    }
    super.updateInputs(inputs);
  }

  private void writeToTable(
      List<PhotonPipelineResult> results, NetworkTable table, Limelight limelight) {
    // write to ll table
    for (PhotonPipelineResult result : results) {
      if (result.getMultiTagResult().isPresent()) {
        Transform3d best = result.getMultiTagResult().get().estimatedPose.best;
        List<Double> pose_data =
            new ArrayList<>(
                Arrays.asList(
                    best.getX(), // 0: X
                    best.getY(), // 1: Y
                    best.getZ(), // 2: Z,
                    best.getRotation().getX(), // 3: roll
                    best.getRotation().getY(), // 4: pitch
                    best.getRotation().getZ(), // 5: yaw
                    result.metadata.getLatencyMillis(), // 6: latency ms,
                    (double)
                        result.getMultiTagResult().get().fiducialIDsUsed.size(), // 7: tag count
                    0.0, // 8: tag span
                    0.0, // 9: tag dist
                    result.getBestTarget().getArea() // 10: tag area
                    ));
        // Add RawFiducials
        // This is super inefficient but it's sim only, who cares.
        for (var target : result.targets) {
          pose_data.add((double) target.getFiducialId()); // 0: id
          pose_data.add(target.getYaw()); // 1: txnc
          pose_data.add(target.getPitch()); // 2: tync
          pose_data.add(0.0); // 3: ta
          pose_data.add(0.0); // 4: distToCamera
          pose_data.add(0.0); // 5: distToRobot
          pose_data.add(0.5); // 6: ambiguity
        }

        table
            .getEntry("botpose_wpiblue")
            .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
        table
            .getEntry("botpose_orb_wpiblue")
            .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
        tagCount[limelight.getId()] = result.getMultiTagResult().get().fiducialIDsUsed.size();
        apriltagDist[limelight.getId()] =
            result.getMultiTagResult().get().estimatedPose.best.getX();
      }

      table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);
      table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
    }
  }

  public PhotonCamera getSimulationCamera(Limelight limelight) {
    return switch (limelight) {
      case SHOOTER -> VisionConstants.SHOOTER_CAMERA;
      case FRONT_LEFT -> VisionConstants.FRONT_LEFT_CAMERA;
      case FRONT_RIGHT -> VisionConstants.FRONT_RIGHT_CAMERA;
      default -> throw new IllegalArgumentException("Invalid limelight camera " + limelight);
    };
  }

  public NetworkTable getLimelightTable(Limelight limelight) {
    return switch (limelight) {
      case SHOOTER -> LimelightHelpers.getLimelightNTTable(Limelight.SHOOTER.getName());
      case FRONT_LEFT -> LimelightHelpers.getLimelightNTTable(Limelight.FRONT_LEFT.getName());
      case FRONT_RIGHT -> LimelightHelpers.getLimelightNTTable(Limelight.FRONT_RIGHT.getName());
      default -> throw new IllegalArgumentException("Invalid limelight " + limelight);
    };
  }

  @Override
  public void setHeadingInfo(double headingDegrees, double headingRateDegrees) {
    super.setHeadingInfo(headingDegrees, headingRateDegrees);
  }

  @Override
  public int getNumberOfAprilTags(Limelight limelight) {
    // TODO Auto-generated method stub
    return tagCount[limelight.getId()];
  }

  @Override
  public double getLimelightAprilTagDistance(Limelight limelight) {
    // TODO Auto-generated method stub
    return apriltagDist[limelight.getId()];
  }

  @Override
  public Pose2d getLastSeenPose() {
    // TODO Auto-generated method stub
    return super.getLastSeenPose();
  }
}
