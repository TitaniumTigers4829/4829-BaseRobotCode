// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class PhysicalArm implements ArmInterface {
  private TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);

  public PhysicalArm() {
    TalonFXConfiguration armConfig = new TalonFXConfiguration();

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armConfig.Slot0.kP = ArmConstants.ARM_P;
    armConfig.Slot0.kI = ArmConstants.ARM_I;
    armConfig.Slot0.kD = ArmConstants.ARM_D;

    armMotor.getConfigurator().apply(armConfig);
  }

  @Override
  public void setAngle(double angle) {
  }
}
