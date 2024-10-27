package frc.robot.subsystems.swerve.moduleIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.extras.simulation.OdometryTimestampsSim;
import frc.robot.extras.simulation.mechanismSim.swervePhysicsSim.SwerveModuleSimulation;
import java.util.Arrays;

/** Wrapper class around {@link SwerveModuleSimulation} that implements ModuleIO */
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
        Units.radiansToRotations(moduleSimulation.getDriveEncoderFinalPositionRad());
    inputs.driveVelocity =
        Units.radiansToRotations(moduleSimulation.getDriveWheelFinalSpeedRadPerSec());
    inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorSupplyCurrentAmps());

    inputs.turnAbsolutePosition = moduleSimulation.getTurnAbsolutePosition();
    // inputs.turnPosition =
    //     Rotation2d.fromRadians(moduleSimulation.getSteerRelativeEncoderPositionRad());
    inputs.turnVelocityRadPerSec = moduleSimulation.getSteerRelativeEncoderSpeedRadPerSec();
    inputs.turnAppliedVolts = moduleSimulation.getSteerMotorAppliedVolts();
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorSupplyCurrentAmps());

    inputs.odometryTimestamps = OdometryTimestampsSim.getTimeStamps();
    inputs.odometryDrivePositionsRad = moduleSimulation.getCachedDriveWheelFinalPositionsRad();
    // inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();

    inputs.odometryDriveWheelRevolutions =
        Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositionsRad())
            .map(Units::radiansToRotations)
            .toArray();
  }

  @Override
  public void setDriveVoltage(double volts) {
    moduleSimulation.requestDriveVoltageOut(volts);
  }

  @Override
  public double getDriveVelocity() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void setTurnPosition(double position) {
    // TODO Auto-generated method stub
    // return 0;
  }

  @Override
  public double getTurnAbsolutePosition() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void setTurnVoltage(double volts) {
    moduleSimulation.requestTurnVoltageOut(volts);
  }

  @Override
  public double getDrivePosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDrivePosition'");
  }

  // public void setDesiredState(SwerveModuleState desiredState) {
  //   Rotation2d turnRotations = getTurnRotations();
  //   // Optimize the reference state to avoid spinning further than 90 degrees
  //   SwerveModuleState optimizedDesiredState =
  //       SwerveModuleState.optimize(desiredState, new Rotation2d(turnRotations));

  //   if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.01) {
  //     moduleSimulation.requestDriveVoltageOut(0);
  //     moduleSimulation.requestTurnVoltageOut(0);
  //     return;
  //   }

  //   // Converts meters per second to rotations per second
  //   double desiredDriveRPS =
  //       optimizedDesiredState.speedMetersPerSecond
  //           * ModuleConstants.DRIVE_GEAR_RATIO
  //           / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

  //   moduleSimulation.requestDriveVoltageOut(
  //       drivePID.calculate(
  //               Units.radiansToRotations(moduleSimulation.getDriveWheelFinalSpeedRadPerSec()),
  //               desiredDriveRPS)
  //           + driveFF.calculate(desiredDriveRPS));
  //   moduleSimulation.requestTurnVoltageOut(
  //       turnPID.calculate(
  //               moduleSimulation.getTurnAbsolutePosition().getRotations(),
  //               desiredState.angle.getRotations())
  //           + turnFF.calculate(turnPID.getSetpoint().velocity));
  // }

  // public Rotation2d getTurnRotations() {
  //   return new Ro(moduleSimulation.getTurnAbsolutePosition()).getRadians();
  // }
}
