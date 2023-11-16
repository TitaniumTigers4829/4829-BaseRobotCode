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

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX exampleMotor;
  private final DoubleSolenoid exampleSolenoid;


  public ExampleSubsystem() {
    //example Configs
    TalonFXConfiguration exampleConfig = new TalonFXConfiguration();
    exampleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    exampleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    exampleConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    exampleConfig.Slot0.kS = DriveConstants.TURN_S;
    exampleConfig.Slot0.kV = DriveConstants.TURN_V;
    exampleMotor.getConfigurator().apply(exampleConfig, HardwareConstants.TIMEOUT_S);

    //configure example solenoid. 
    exampleSolenoid = new DoubleSolenoid(ElectronicsConstants.pneumaticsModuleType, IntakeConstants.deploySolenoidPort, IntakeConstants.retractSolenoidPort);

    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
  }
  public void setMotorStopped() {
    exampleMotor.set(0);
  }
  public void setMotorFullPowerIn() {
    exampleMotor.set(1);
  }
  public void setMotorFullPowerOut() {
    exampleMotor.set(-1);
  }
  public void setCustomPower(double customPower) {
    exampleMotor.set(customPower);
  }


  public void setRetractedSolenoidPort() {
    exampleSolenoid.set(Value.kReverse);
  }
  public void setDeployedSolenoidPort() {
    exampleSolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
