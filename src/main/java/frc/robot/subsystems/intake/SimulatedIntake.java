package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimulatedIntake implements IntakeInterface {
  DCMotorSim intakeSim = new DCMotorSim(null, DCMotor.getFalcon500(1), 0);
  SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(2), IntakeConstants.INTAKE_PIVOT_GEAR_RATIO, 0, 0, 0, 0, false, 0);
  private final Constraints pivotConstraints = new Constraints(0, 0);
  private final ProfiledPIDController pivotController =
      new ProfiledPIDController(0, 0, 0, pivotConstraints);

  private final ArmFeedforward pivotFF = new ArmFeedforward(0, 0, 0, 0);

  private double intakeAppliedVolts = 0.0;
  private double pivotAppliedVolts = 0.0;

  public SimulatedIntake(IntakeInputs inputs) {
    intakeSim.update(0.02);
    pivotSim.update(0.02);

    inputs.intakeVelocity = intakeSim.getAngularVelocityRadPerSec() / (Math.PI * 2);
    inputs.intakeCurrentAmps = intakeSim.getCurrentDrawAmps();
    inputs.intakeAppliedVolts = intakeAppliedVolts;

    inputs.pivotPosition = pivotSim.getAngleRads() / (Math.PI * 2);
    inputs.pivotVelocity = pivotSim.getVelocityRadPerSec() / (Math.PI * 2);
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();
  }

  @Override
  public void setPivotPosition(double position) {
    pivotSim.setInputVoltage(
        pivotController.calculate(position, pivotSim.getAngleRads() / (Math.PI * 2))
            + pivotFF.calculate(
                pivotController.getSetpoint().position, pivotController.getSetpoint().velocity));
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeSim.setInputVoltage(speed);
  }

  @Override
  public double getIntakeSpeed() {
    return intakeSim.getAngularVelocityRadPerSec() / (Math.PI * 2);
  }

  @Override
  public double getPivotPosition() {
    return pivotSim.getAngleRads() / (Math.PI * 2);
  }
}