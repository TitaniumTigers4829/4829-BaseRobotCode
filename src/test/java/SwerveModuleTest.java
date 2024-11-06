import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.moduleIO.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.moduleIO.ModuleInterface;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SwerveModuleTest {

  private SwerveModule swerveModule;
  private ModuleInterface mockModuleInterface;
  private ModuleInputsAutoLogged mockInputs;

  @BeforeEach
  void setUp() {
    // Mock the ModuleInterface and inputs
    mockModuleInterface = mock(ModuleInterface.class);
    mockInputs = mock(ModuleInputsAutoLogged.class);

    // Initialize SwerveModule with the mocked interface
    swerveModule = new SwerveModule(mockModuleInterface, "FrontLeft");
  }

  @Test
  void testConstructor() {
    // Verify that the constructor correctly initializes the name and hardware fault alert
    assertNotNull(swerveModule);
    assertEquals("Module-FrontLeft", swerveModule.getName());
    // assertNotNull(swerveModule.hardwareFaultAlert);
    // assertFalse(swerveModule.hardwareFaultAlert.isSet());
  }

  @Test
  void testUpdateOdometryInputs() {
    // Mock the inputs being updated
    doNothing().when(mockModuleInterface).updateInputs(mockInputs);

    // Test updateOdometryInputs method
    swerveModule.updateOdometryInputs();

    // Verify that the inputs were updated and logged
    verify(mockModuleInterface).updateInputs(mockInputs);
    // verify(swerveModule.hardwareFaultAlert).set(false); // Hardware fault alert should be false
    // if inputs are connected
  }

  @Test
  void testSetVoltage() {
    // Test setting the voltage to the module
    Voltage volts = Volts.of(12.0);
    swerveModule.setVoltage(volts);

    // Verify that the voltage is set to the drive
    verify(mockModuleInterface).setDriveVoltage(volts);
    // Ensure the turn voltage is set to zero
    verify(mockModuleInterface).setTurnVoltage(Volts.zero());
  }

  @Test
  void testGetDriveVoltage() {
    // Mock the drive voltage input
    when(mockInputs.driveAppliedVolts).thenReturn(5.0);

    // Test the getDriveVoltage method
    double driveVoltage = swerveModule.getDriveVoltage();
    assertEquals(5.0, driveVoltage, 0.001);
  }

  @Test
  void testGetCharacterizationVelocity() {
    // Mock the velocity input
    when(mockInputs.driveVelocity).thenReturn(3.0);

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
  void testGetTurnRotation() {
    // Mock the turn angle input
    when(mockInputs.turnAbsolutePosition).thenReturn(Rotation2d.fromDegrees(90));

    // Test the getTurnRotation method
    Rotation2d rotation = swerveModule.getTurnRotation();
    assertEquals(90.0, rotation.getDegrees(), 0.001);
  }

  @Test
  void testGetDrivePositionMeters() {
    // Mock the drive position input
    when(mockInputs.drivePosition).thenReturn(2.0);

    // Test the getDrivePositionMeters method
    double drivePosition = swerveModule.getDrivePositionMeters();
    assertEquals(2.0, drivePosition, 0.001);
  }

  @Test
  void testGetDriveVelocityMetersPerSec() {
    // Mock the drive velocity input
    when(mockInputs.driveVelocity).thenReturn(4.0);

    // Test the getDriveVelocityMetersPerSec method
    double driveVelocity = swerveModule.getDriveVelocityMetersPerSec();
    assertEquals(4.0, driveVelocity, 0.001);
  }

  @Test
  void testGetMeasuredState() {
    // Mock the drive velocity and turn rotation
    when(mockInputs.driveVelocity).thenReturn(3.0);
    when(mockInputs.turnAbsolutePosition).thenReturn(Rotation2d.fromDegrees(180));

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
    when(swerveModule.getOdometryPositions()).thenReturn(mockPositions);

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
    when(mockInputs.drivePosition).thenReturn(1.5);
    when(mockInputs.turnAbsolutePosition).thenReturn(Rotation2d.fromDegrees(45));

    // Verify that the getPosition method returns the correct position
    SwerveModulePosition position = swerveModule.getPosition();
    assertEquals(1.5, position.distanceMeters, 0.001);
    assertEquals(45.0, position.angle.getDegrees(), 0.001);
  }
}
