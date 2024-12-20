package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public class Arm {
  private ArmInterface armInterface;
  private ArmInputsAutoLogged input = new ArmInputsAutoLogged();
  
  public Arm(ArmInterface armInterface) {
    this.armInterface = armInterface;
  }
}
