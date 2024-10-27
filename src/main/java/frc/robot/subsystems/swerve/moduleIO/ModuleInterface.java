package frc.robot.subsystems.swerve.moduleIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleInterface {
  @AutoLog
  class ModuleInputs {

    public boolean isConnected = false;
    // public Rotation2d turnRotation = new Rotation2d();
    public double steerVelocityRadPerSec = 0.0;
    public double steerMotorAppliedVolts = 0.0;
    public double steerMotorCurrentAmps = 0.0;

    public double[] odometryDriveWheelRevolutions = new double[] {};

    public double driveVelocity = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double drivePosition = 0.0;
    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    // public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    public double driveWheelFinalVelocityPerSec = 0.0;
    public double turnMotorAppliedVolts = 0.0;
    public double turnMotorCurrentAmps = 0.0;
    public double turnPosition = 0.0;
  }

  /** Updates the inputs */
  default void updateInputs(ModuleInputs inputs) {}

  default String getCANBus() {
    return "";
  }

  default void calibrate() {}

  default void setDesiredState(SwerveModuleState desiredState) {}

  double getDriveVelocity();

  void setTurnPosition(double position);

  double getTurnAbsolutePosition();

  /**
   * Run the drive motor at the specified percent speed.
   *
   * @param speedPercent from -1 to 1, where 1 is the forward direction of the wheel
   */
  default void setDriveSpeed(double speedPercent) {}

  /**
   * Run the turn motor at the specified percent power.
   *
   * @param powerPercent from -1 to 1, where 1 is counter-clockwise
   */
  default void setTurnSpeed(double powerPercent) {}

  /** Enable or disable brake mode on the drive motor. */
  default void setDriveBrake(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  default void setTurnBrake(boolean enable) {}

  default void setDriveVoltage(double voltage) {}

  default void setTurnVoltage(double voltage) {}

  default void stopModule() {}

  default double getTurnRotations() {
    return 0.0;
  }

  default double getDriveVoltage() {
    return 0.0;
  }

  double getDrivePosition();
}
