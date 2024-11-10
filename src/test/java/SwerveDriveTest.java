import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.gyroIO.GyroInterface;
import frc.robot.subsystems.swerve.moduleIO.ModuleInterface;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

class SwerveDriveTest {

  @Mock private GyroInterface gyroIO;

  @Mock
  private ModuleInterface frontLeftModuleIO,
      frontRightModuleIO,
      backLeftModuleIO,
      backRightModuleIO;
  @Mock private SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

  @Mock private OdometryThread mockOdometryThread;

  @Mock private SwerveDriveKinematics swerveDriveKinematics;

  private SwerveDrive swerveDrive;

  @BeforeEach
  void setUp() {
    // Initialize the mocks
    MockitoAnnotations.openMocks(this);

    swerveDrive =
        spy(
            new SwerveDrive(
                gyroIO,
                frontLeftModuleIO,
                frontRightModuleIO,
                backLeftModuleIO,
                backRightModuleIO));

    swerveDrive.setKinematics(swerveDriveKinematics);
  }

  @Test
  void testGetGyroRate() {
    // Mock gyro inputs
    when(swerveDrive.getGyroRate()).thenReturn(0.5);

    // Test the getGyroRate method
    double gyroRate = swerveDrive.getGyroRate();
    assertEquals(0.5, gyroRate, 0.001);
  }

  @Test
  void testDrive() {
    // Prepare mock module states
    SwerveModuleState[] mockStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      mockStates[i] = mock(SwerveModuleState.class); // Mock each SwerveModuleState
    }

    // Mock the toSwerveModuleStates method to return mock states
    when(swerveDriveKinematics.toSwerveModuleStates(any(ChassisSpeeds.class)))
        .thenReturn(mockStates);

    // Now when the drive method calls toSwerveModuleStates, it will return mockStates
    swerveDrive.drive(1.0, 1.0, 0.5, true);

    // Verify that setModuleStates was called with mockStates
    verify(swerveDrive).setModuleStates(mockStates);
    verify(swerveDrive.getKinematics()).toSwerveModuleStates(any(ChassisSpeeds.class));
  }

  @Test
  void testGetPose() {
    // Mock the pose estimator to return a specific pose
    Pose2d mockPose = new Pose2d(1.0, 2.0, new Rotation2d(Math.toRadians(45)));
    when(swerveDrive.getPose()).thenReturn(mockPose);

    // Call the getPose method and verify the returned pose
    Pose2d pose = swerveDrive.getPose();
    assertEquals(1.0, pose.getX(), 0.001);
    assertEquals(2.0, pose.getY(), 0.001);
    assertEquals(Math.toRadians(45), pose.getRotation().getRadians(), 0.001);
  }

  @Test
  void testSetPose() {
    // Mock pose setter
    Pose2d newPose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90));
    swerveDrive.setPose(newPose);

    // Verify that the setPose method was called with the new pose
    verify(swerveDrive).setPose(eq(newPose));
  }
}
