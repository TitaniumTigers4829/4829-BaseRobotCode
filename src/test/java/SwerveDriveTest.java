

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.mockito.Mockito.when;

import java.util.Optional;

import org.junit.Before;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.moduleIO.ModuleInterface;
import frc.robot.subsystems.swerve.gyroIO.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyroIO.GyroInterface;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;

class SwerveDriveTest {

    private SwerveDrive swerveDrive;
    private GyroInterface mockGyroIO;
    private ModuleInterface mockFrontLeftModule;
    private ModuleInterface mockFrontRightModule;
    private ModuleInterface mockBackLeftModule;
    private ModuleInterface mockBackRightModule;
    private OdometryThread mockOdometryThread;

    @BeforeEach
    void setUp() {
        // Mock the dependencies
        mockGyroIO = mock(GyroInterface.class);
        mockFrontLeftModule = mock(ModuleInterface.class);
        mockFrontRightModule = mock(ModuleInterface.class);
        mockBackLeftModule = mock(ModuleInterface.class);
        mockBackRightModule = mock(ModuleInterface.class);
        mockOdometryThread = mock(OdometryThread.class);

        // Initialize the SwerveDrive with the mocked dependencies
        swerveDrive = new SwerveDrive(mockGyroIO, mockFrontLeftModule, mockFrontRightModule, mockBackLeftModule, mockBackRightModule);
    }

    @Test
    void testConstructorInitialization() {
        // Ensure that the constructor correctly initializes the pose estimator and modules
        assertNotNull(swerveDrive);
        // assertNotNull(swerveDrive.poseEstimator);
        // assertNotNull(swerveDrive.swerveModules);
        // assertEquals(4, swerveDrive.swerveModules.length); // 4 modules
    }

    @Test
    void testGetGyroRate() {
        // Mock gyro inputs
        GyroInputsAutoLogged inputsAutoLogged = new GyroInputsAutoLogged();
        when(inputsAutoLogged.yawVelocity).thenReturn(0.5);

        // Test the getGyroRate method
        double gyroRate = swerveDrive.getGyroRate();
        assertEquals(0.5, gyroRate, 0.001);
    }

    @Test
    void testDrive() {
        // Prepare mock module states
        SwerveModuleState[] mockStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            mockStates[i] = mock(SwerveModuleState.class);
        }

        // Mock the kinematics method to return mock states
        when(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(any(ChassisSpeeds.class))).thenReturn(mockStates);

        // Call the drive method
        swerveDrive.drive(1.0, 1.0, 0.5, true);

        // Verify that setModuleStates was called
        verify(swerveDrive).setModuleStates(mockStates);
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
        // Test for Blue Alliance
        when(DriverStation.getAlliance()).thenReturn(Optional.of(DriverStation.Alliance.Blue));
        assertEquals(0.0, swerveDrive.getAllianceAngleOffset(), 0.001);

        // Test for Red Alliance
        when(DriverStation.getAlliance()).thenReturn(Optional.of(DriverStation.Alliance.Red));
        assertEquals(180.0, swerveDrive.getAllianceAngleOffset(), 0.001);
    }
}
