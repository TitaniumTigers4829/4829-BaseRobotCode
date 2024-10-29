package frc.robot.subsystems.swerve.moduleIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleInterface {
  @AutoLog
  class ModuleInputs {

    public boolean isConnected;

    public double driveVelocity;
    public double driveAppliedVolts;
    public double driveCurrentAmps;
    public double drivePosition;

    public Rotation2d turnAbsolutePosition;
    public double turnVelocity;
    public double turnAppliedVolts;
    public double turnCurrentAmps;

    public double[] odometryTimestamps;

    public double turnPosition;
  }

  /**
   * Updates the inputs created in ModuleInputs
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

  void setDriveVoltage(Voltage voltage);

  void setTurnVoltage(Voltage voltage);

  void stopModule();

  double getTurnRotations();
}
