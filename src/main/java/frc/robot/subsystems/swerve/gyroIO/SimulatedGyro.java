package frc.robot.subsystems.swerve.gyroIO;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.robot.extras.simulation.OdometryTimestampsSim;
import frc.robot.extras.simulation.mechanismSim.swerve.GyroSimulation;

public class SimulatedGyro implements GyroInterface {
  private final GyroSimulation gyroSimulation;

  public SimulatedGyro(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.isConnected = true;
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    inputs.odometryYawTimestamps = OdometryTimestampsSim.getTimestamps();
    inputs.yawDegreesRotation2d = gyroSimulation.getGyroReading();
    inputs.yawVelocity =
        RadiansPerSecond.of(gyroSimulation.getMeasuredAngularVelocityRadPerSec())
            .in(DegreesPerSecond);
  }
}
