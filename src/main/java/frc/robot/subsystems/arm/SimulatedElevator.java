// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class SimulatedElevator implements ArmInterface {
  private SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), ArmConstants.ARM_GEAR_RATIO, ArmConstants.ARM_INERTIA_MASS, ArmConstants.ARM_LENGTH_METERS, ArmConstants.MINIMUM_ANGLE_RADIANS, ArmConstants.MAXIMUM_ANGLE_RADIANS, false, 0, null);
  private PIDController pidSim;

  public SimulatedElevator() {
    pidSim = new PIDController(ArmConstants.ARM_P, ArmConstants.ARM_I, ArmConstants.ARM_D);
  }
}