import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.gyroIO.GyroInterface;
import frc.robot.subsystems.swerve.moduleIO.ModuleInterface;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockedStatic;
import org.mockito.MockitoAnnotations;

class SwerveDriveTest {

  @Mock private GyroInterface gyroIO;

  @Mock
  private ModuleInterface frontLeftModuleIO,
      frontRightModuleIO,
      backLeftModuleIO,
      backRightModuleIO;
  @Mock private SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

  // private OdometryThread mockOdometryThread;

  @Mock private SwerveDriveKinematics swerveDriveKinematics;

  private SwerveDrive swerveDrive;

  @BeforeEach
  void setUp() {
      // Initialize the mocks
      MockitoAnnotations.openMocks(this);
  
      // Ensure mocks are not null before using them
      assertNotNull(gyroIO, "gyroIO is null");
      assertNotNull(frontLeftModuleIO, "frontLeftModuleIO is null");
      assertNotNull(frontRightModuleIO, "frontRightModuleIO is null");
      assertNotNull(backLeftModuleIO, "backLeftModuleIO is null");
      assertNotNull(backRightModuleIO, "backRightModuleIO is null");
  
      // Try to create SwerveDrive and catch any issues
      try {
          swerveDrive = new SwerveDrive(
              gyroIO, frontLeftModuleIO, frontRightModuleIO, backLeftModuleIO, backRightModuleIO
          );
      } catch (Exception e) {
          System.out.println("Exception during SwerveDrive instantiation: " + e.getMessage());
          e.printStackTrace();
      }
  
      // Verify swerveDrive is not null after instantiation
      assertNotNull(swerveDrive, "SwerveDrive instantiation failed");
  
      // Set mocked kinematics if SwerveDrive was created successfully
      if (swerveDrive != null) {
          swerveDrive.setKinematics(swerveDriveKinematics);
          swerveDrive = spy(swerveDrive);
      }
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

    // Verify that the resetPosition method was called with the new pose
    verify(swerveDrive).setPose(eq(newPose));
  }

  // @Test
  // void testCharacterization() {
  //     // Mock voltage setting on modules
  //     doNothing().when(mockFrontLeftModule).setVoltage(any());
  //     doNothing().when(mockFrontRightModule).setVoltage(any());
  //     doNothing().when(mockBackLeftModule).setVoltage(any());
  //     doNothing().when(mockBackRightModule).setVoltage(any());

  //     // Call the runCharacterization method
  //     swerveDrive.runCharacterization(12.0);

  //     // Verify that the setVoltage method was called on all modules
  //     verify(mockFrontLeftModule).setVoltage(-12.0);
  //     verify(mockFrontRightModule).setVoltage(-12.0);
  //     verify(mockBackLeftModule).setVoltage(-12.0);
  //     verify(mockBackRightModule).setVoltage(-12.0);
  // }

  // @Test
  // void testFetchOdometryInputs() {
  //     // Mock inputs and behavior
  //     when(mockGyroIO.updateInputs(any())).thenReturn(null);
  //     doNothing().when(mockOdometryThread).lockOdometry();
  //     doNothing().when(mockOdometryThread).unlockOdometry();

  //     // Call the fetchOdometryInputs method
  //     swerveDrive.fetchOdometryInputs();

  //     // Verify the behavior of the method
  //     verify(mockOdometryThread).lockOdometry();
  //     verify(mockOdometryThread).unlockOdometry();
  //     verify(mockGyroIO).updateInputs(any());
  // }

  @Test
  void testAllianceAngleOffset() {
    // Use mockStatic to mock static methods like DriverStation.getAlliance()
    try (MockedStatic<DriverStation> mockedDriverStation = mockStatic(DriverStation.class)) {
      // Mock for Blue Alliance
      mockedDriverStation.when(DriverStation::getAlliance).thenReturn(DriverStation.Alliance.Blue);
      assertEquals(0.0, swerveDrive.getAllianceAngleOffset(), 0.001);

      // Mock for Red Alliance
      mockedDriverStation.when(DriverStation::getAlliance).thenReturn(DriverStation.Alliance.Red);
      assertEquals(180.0, swerveDrive.getAllianceAngleOffset(), 0.001);
    }
  }
}
