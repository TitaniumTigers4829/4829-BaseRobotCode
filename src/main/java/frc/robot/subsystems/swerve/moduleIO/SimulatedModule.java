package frc.robot.subsystems.swerve.moduleIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.extras.simulation.OdometryTimestampsSim;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveModuleSimulation;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

/** Wrapper class around {@link SwerveModuleSimulation} */
public class SimulatedModule implements ModuleInterface {
  private final SwerveModuleSimulation moduleSimulation;

  private final PIDController drivePID = new PIDController(5, 0, 0);
  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(5, 0, 0);
  private final Constraints turnConstraints = new Constraints(5, 0);
  private final ProfiledPIDController turnPID = new ProfiledPIDController(5, 0, 0, turnConstraints);
  private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(5, 0, 0);

  public SimulatedModule(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {
    inputs.drivePosition =
        Radians.of(moduleSimulation.getDriveEncoderFinalPositionRad()).in(Rotations);
    inputs.driveVelocity =
        RadiansPerSecond.of(moduleSimulation.getDriveWheelFinalSpeedRadPerSec())
            .in(RotationsPerSecond);
    inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorSupplyCurrentAmps());

    // inputs.turnAbsolutePosition = moduleSimulation.getTurnAbsolutePosition();
    inputs.turnPosition =
        Radians.of(moduleSimulation.getTurnRelativeEncoderPositionRad()).in(Rotations);
    inputs.turnVelocity =
        RadiansPerSecond.of(moduleSimulation.getTurnRelativeEncoderSpeedRadPerSec())
            .in(RotationsPerSecond);
    inputs.turnAppliedVolts = moduleSimulation.getTurnMotorAppliedVolts();
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getTurnMotorSupplyCurrentAmps());

    inputs.odometryTimestamps = OdometryTimestampsSim.getTimestamps();
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    moduleSimulation.requestDriveVoltageOut(volts);
  }

  @Override
  public void setTurnVoltage(Voltage volts) {
    moduleSimulation.requestTurnVoltageOut(volts);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    double turnRotations = getTurnRotations();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState setpoint =
        new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);

    setpoint.optimize(Rotation2d.fromRotations(turnRotations));
    setpoint.cosineScale(Rotation2d.fromRotations(turnRotations));

    if (Math.abs(setpoint.speedMetersPerSecond) < 0.01) {
      moduleSimulation.requestDriveVoltageOut(0);
      moduleSimulation.requestTurnVoltageOut(0);
      return;
    }

    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        setpoint.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    moduleSimulation.requestDriveVoltageOut(
        Volts.of(
                drivePID.calculate(
                    Units.radiansToRotations(moduleSimulation.getDriveWheelFinalSpeedRadPerSec()),
                    desiredDriveRPS))
            .plus(driveFF.calculate(RotationsPerSecond.of(desiredDriveRPS))));
    moduleSimulation.requestTurnVoltageOut(
        Volts.of(
                turnPID.calculate(
                    moduleSimulation.getTurnAbsolutePosition().getRotations(),
                    desiredState.angle.getRotations()))
            .plus(turnFF.calculate(RotationsPerSecond.of(turnPID.getSetpoint().velocity))));
  }

  public double getTurnRotations() {
    return moduleSimulation.getTurnAbsolutePosition().getRotations();
  }

  @Override
  public void stopModule() {
    moduleSimulation.requestDriveVoltageOut(Volts.zero());
    moduleSimulation.requestTurnVoltageOut(Volts.zero());
  }
}
