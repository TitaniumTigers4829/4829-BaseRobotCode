package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import static edu.wpi.first.units.Units.*;


public class SimulatedIntake implements IntakeInterface {
  DCMotorSim intakeSim = new DCMotorSim(null, DCMotor.getFalcon500(1), 0);
  private double intakeAppliedVolts = 0.0;

  public SimulatedIntake(IntakeInputs inputs) {
    intakeSim.update(0.02);
  
    inputs.intakeVelocity = RadiansPerSecond.of(intakeSim.getAngularVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.intakeCurrentAmps = intakeSim.getCurrentDrawAmps();
    inputs.intakeAppliedVolts = intakeAppliedVolts;
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeSim.setInputVoltage(speed);
  }

  @Override
  public double getIntakeSpeed() {
    return RadiansPerSecond.of(intakeSim.getAngularVelocityRadPerSec()).in(RotationsPerSecond);
  }
}