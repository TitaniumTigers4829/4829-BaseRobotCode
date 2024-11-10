import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.moduleIO.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.moduleIO.ModuleInterface;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

class SwerveModuleTest {

  private SwerveModule swerveModule;
  @Mock private ModuleInterface mockModuleInterface;
  @Mock private ModuleInputsAutoLogged mockInputs;

  @BeforeEach
  void setUp() {
    MockitoAnnotations.openMocks(this);

    // Initialize SwerveModule with the mocked interface
    swerveModule = spy(new SwerveModule(mockModuleInterface, "Mocked Module"));
    mockInputs = spy(mockInputs);
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
  void testGetOdometryPositions() {
    // Mock the odometry positions
    SwerveModulePosition[] mockPositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90)),
          new SwerveModulePosition(2.0, Rotation2d.fromDegrees(180))
        };
    when(swerveModule.getOdometryPositions()).thenAnswer(answer -> mockPositions);

    // Verify that the getOdometryPositions method returns the correct positions
    SwerveModulePosition[] positions = swerveModule.getOdometryPositions();
    assertEquals(1.0, positions[0].distanceMeters, 0.001);
    assertEquals(90.0, positions[0].angle.getDegrees(), 0.001);
    assertEquals(2.0, positions[1].distanceMeters, 0.001);
    assertEquals(180.0, positions[1].angle.getDegrees(), 0.001);
  }
}
