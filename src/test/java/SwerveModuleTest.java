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
  void testRunSetpoint() {
    // Prepare a mock state for the module
    SwerveModuleState mockState = new SwerveModuleState(2.0, Rotation2d.fromDegrees(45));

    // Call runSetPoint and verify that the setDesiredState method is called on the module interface
    swerveModule.runSetpoint(mockState);
    verify(mockModuleInterface).setDesiredState(mockState);
  }
}
