// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.ElectronicsConstants;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonFX motor;
  private final DoubleSolenoid solenoid;


  public IntakeSubsystem() {
    motor = new WPI_TalonFX(IntakeConstants.intakeMotorPort);
    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);

    solenoid = new DoubleSolenoid(ElectronicsConstants.pneumaticsModuleType, IntakeConstants.deploySolenoidPort, IntakeConstants.retractSolenoidPort);

    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
  }
  public void setMotorStopped() {
    motor.set(0);
  }
  public void setMotorFullPowerIn() {
    motor.set(1);
  }
  public void setMotorFullPowerOut() {
    motor.set(-1);
  }
  public void setCustomPower(double customPower) {
    motor.set(customPower);
  }


  public void setRetractedSolenoidPort() {
    solenoid.set(Value.kReverse);
  }
  public void setDeployedSolenoidPort() {
    solenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
