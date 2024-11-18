// package frc.robot.subsystems.vision;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.geometry.Pose2d;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.VisionSystemSim;

// import java.util.List;
// import java.util.function.Supplier;

// public class VisionIOSim extends VisionIOPhoton {
//     private final VisionSystemSim visionSystemSim;
//     private final PhotonCameraSim[] camerasSim;
//     private final Supplier<Pose2d> robotActualPoseInSimulationSupplier;
//     public VisionIOSim(List<PhotonCameraProperties> cameraProperties, AprilTagFieldLayout
// aprilTagFieldLayout, Supplier<Pose2d> robotActualPoseInSimulationSupplier) {
//         super(cameraProperties);

//         this.robotActualPoseInSimulationSupplier = robotActualPoseInSimulationSupplier;
//         this.visionSystemSim = new VisionSystemSim("main");
//         visionSystemSim.addAprilTags(aprilTagFieldLayout);
//         camerasSim = new PhotonCameraSim[cameraProperties.size()];

//         for (int i = 0; i < cameraProperties.size(); i++) {
//             final PhotonCameraSim cameraSim = new PhotonCameraSim(
//                     super.cameras[i],
//                     cameraProperties.get(i).getSimulationProperties()
//             );
//             cameraSim.enableRawStream(true);
//             cameraSim.enableProcessedStream(true);
//             cameraSim.enableDrawWireframe(true);
//             visionSystemSim.addCamera(
//                     camerasSim[i] = cameraSim,
//                     cameraProperties.get(i).robotToCamera
//             );
//         }
//     }

//     @Override
//     public void updateInputs(VisionIOInputs inputs) {
//         visionSystemSim.update(robotActualPoseInSimulationSupplier.get());
//         super.updateInputs(inputs);
//     }
// }
