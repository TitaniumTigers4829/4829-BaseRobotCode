package frc.robot.subsystems.swerve.moduleIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleInterface {
  @AutoLog
  class ModuleInputs {

    public boolean isConnected = false;

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
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    public double driveWheelFinalVelocityPerSec = 0.0;
    public double turnMotorAppliedVolts = 0.0;
    public double turnMotorCurrentAmps = 0.0;
    public double turnPosition = 0.0;
  }

  /**
   * Updates the inputs
   *
   * @param inputs the inputs to update
   */
  void updateInputs(ModuleInputs inputs);

  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   *
   * @param desiredState Desired state with speed and angle.
   */
  void setDesiredState(SwerveModuleState desiredState);

  double getDriveVelocity();

  double getTurnAbsolutePosition();

  void setDriveVoltage(Voltage voltage);

  void setTurnVoltage(Voltage voltage);

  void stopModule();

  double getTurnRotations();

  double getDriveVoltage();

  double getDrivePosition();
}
