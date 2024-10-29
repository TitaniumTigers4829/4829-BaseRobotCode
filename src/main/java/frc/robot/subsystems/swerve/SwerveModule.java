package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.moduleIO.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.moduleIO.ModuleInterface;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {
  private final ModuleInterface io;
  private final String name;
  private final ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  private final Alert hardwareFaultAlert;

  public SwerveModule(ModuleInterface io, String name) {
    super("Module-" + name);
    this.io = io;
    this.name = name;
    this.hardwareFaultAlert =
        new Alert("Module-" + name + " Hardware Fault", Alert.AlertType.kError);
    this.hardwareFaultAlert.set(false);

    CommandScheduler.getInstance().unregisterSubsystem(this);
  }

  public void updateOdometryInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module-" + name, inputs);
    this.hardwareFaultAlert.set(!inputs.isConnected);
  }

  @Override
  public void periodic() {}

  public void setVoltage(Voltage volts) {
    io.setDriveVoltage(volts);
    io.setTurnVoltage(Volts.zero());
  }

  public double getDriveVoltage() {
    return inputs.driveAppliedVolts;
  }

  public double getCharacterizationVelocity() {
    return inputs.driveVelocity;
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public void runSetPoint(SwerveModuleState state) {
    io.setDesiredState(state);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getTurnRotation() {
    return inputs.turnAbsolutePosition;
  }

  public double getTurnVelocity() {
    return inputs.turnVelocity;
  }

  /** Returns the current drive position of the module in meters. */
  public double getDrivePositionMeters() {
    return ModuleConstants.DRIVE_TO_METERS * inputs.drivePosition;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getDriveVelocityMetersPerSec() {
    return ModuleConstants.DRIVE_TO_METERS_PER_SECOND * inputs.driveVelocity;
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSec(), getTurnRotation());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /**
   * Gets the module position consisting of the distance it has traveled and the angle it is
   * rotated.
   *
   * @return a SwerveModulePosition object containing position and rotation
   */
  public SwerveModulePosition getPosition() {
    double position = ModuleConstants.DRIVE_TO_METERS * getDrivePositionMeters();
    Rotation2d rotation = getTurnRotation();
    return new SwerveModulePosition(position, rotation);
  }
}
