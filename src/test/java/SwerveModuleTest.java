import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.moduleIO.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.moduleIO.ModuleInterface;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import com.fasterxml.jackson.databind.Module;

class SwerveModuleTest {

  private SwerveModule swerveModule;
  @Mock
  private ModuleInterface mockModuleInterface;
  @Mock
  private ModuleInputsAutoLogged mockInputs;

  @BeforeEach
  void setUp() {
    MockitoAnnotations.openMocks(this);

    // Initialize SwerveModule with the mocked interface
    swerveModule = new SwerveModule(mockModuleInterface, "Mocked Module");
    swerveModule = spy(swerveModule);
    assertNotNull(swerveModule);
    mockInputs = spy(mockInputs);
    assertNotNull(mockInputs);

  }

  @Test
  void testUpdateOdometryInputs() {
    // Mock the inputs being updated
    // when(mockModuleInterface).updateInputs(mockInputs);

    // Test updateOdometryInputs method
    swerveModule.updateOdometryInputs();

    // Verify that the inputs were updated and logged
    verify(mockModuleInterface).updateInputs(mockInputs);
  }

  @Test
  void testGetCharacterizationVelocity() {
    // Mock the velocity input
    when(mockInputs.driveVelocity).thenAnswer(answer -> 3.0);

    // Test the getCharacterizationVelocity method
    double velocity = swerveModule.getCharacterizationVelocity();
    assertEquals(3.0, velocity, 0.001);
  }

  @Test
  void testRunSetPoint() {
    // Prepare a mock state for the module
    SwerveModuleState mockState = new SwerveModuleState(2.0, Rotation2d.fromDegrees(45));

    // Call runSetPoint and verify that the setDesiredState method is called on the module interface
    swerveModule.runSetPoint(mockState);
    verify(mockModuleInterface).setDesiredState(mockState);
  }

  @Test
  void testGetMeasuredState() {
    // Mock the drive velocity and turn rotation
    when(mockInputs.driveVelocity).thenAnswer(answer -> 3.0);
    when(mockInputs.turnAbsolutePosition).thenAnswer(answer -> Rotation2d.fromDegrees(180));

    // Get the measured state and verify the values
    SwerveModuleState state = swerveModule.getMeasuredState();
    assertEquals(3.0, state.speedMetersPerSecond, 0.001);
    assertEquals(180.0, state.angle.getDegrees(), 0.001);
  }

  @Test
  void testGetOdometryPositions() {
    // Mock the odometry positions
    SwerveModulePosition[] mockPositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90)),
          new SwerveModulePosition(2.0, Rotation2d.fromDegrees(180))
        };
    when(swerveModule.getOdometryPositions()).thenAnswer(t -> mockPositions);

    // Verify that the getOdometryPositions method returns the correct positions
    SwerveModulePosition[] positions = swerveModule.getOdometryPositions();
    assertEquals(1.0, positions[0].distanceMeters, 0.001);
    assertEquals(90.0, positions[0].angle.getDegrees(), 0.001);
    assertEquals(2.0, positions[1].distanceMeters, 0.001);
    assertEquals(180.0, positions[1].angle.getDegrees(), 0.001);
  }

  @Test
  void testGetPosition() {
    // Mock the inputs for position and rotation
    doReturn(1.5).doAnswer(t -> mockInputs.drivePosition);
    // doReturn(Rotation2d.fromDegrees(45)).doAnswer(t -> mockInputs.turnAbsolutePosition);

    // Verify that the getPosition method returns the correct position
    SwerveModulePosition position = swerveModule.getPosition();
    assertEquals(1.5, position.distanceMeters, 0.001);
    assertEquals(45.0, position.angle.getDegrees(), 0.001);
  }
}
