package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.elevator.ElevatorInterface.ElevatorInputs;

/** Add your docs here. */
public class PhysicalElevator {
  private TalonFX leaderMotor = new TalonFX(0);
  private TalonFX followerMotor = new TalonFX(0);

  StatusSignal<Angle> leaderPosition;
  Statussignal<Angle> followerPosition;
  public PhysicalElevator() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    elevatorConfig.Slot0.kP = 0;
    elevatorConfig.Slot0.kI = 0;
    elevatorConfig.Slot0.kD = 0;

    leaderMotor.getConfigurator().apply(elevatorConfig);
    followerMotor.getConfigurator().apply(elevatorConfig);
  }

  public void updateInputs(ElevatorInputs inputs) {
    inputs.leaderMotorPosition = leaderPosition.getValueAsDouble();
    inputs.leaderMotorVelocity = leaderVelocity.getValueAsDouble();
    inputs.leaderMotorAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderMotorCurrentAmps = leaderCurrentAmps.getValueAsDouble();

    inputs.followerMotorPosition = followerPosition.getValueAsDouble();
    inputs.followerMotorVelocity = followerVelocity.getValueAsDouble();
    inputs.followerMotorAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.followerMotorCurrentAmps = followerCurrentAmps.getValueAsDouble();
  }

  public double getElevatorPosition() {
    return leaderMotor.getRotorPosition().getValueAsDouble();
  }

  public void setElevatorPosition(double position) {
    leaderMotor.setPosition(position);
    followerMotor.setPosition(position);
  }

  public void setElevatorSpeed(double speed) {
    leaderMotor.set(speed);
    followerMotor.set(speed);
  }

  public void setVolts(double volts) {
    leaderMotor.setVoltage(volts);
    followerMotor.setVoltage(volts);;
  }

  public double getVolts() {
    return leaderMotor.getMotorVoltage().getValueAsDouble();
  }
}
