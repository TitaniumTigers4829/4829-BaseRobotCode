package frc.robot.subsystems.arm;

public class Arm {
  private ArmInterface armInterface;
  private ArmInputsAutoLogged input = new ArmInputsAutoLogged();

  public Arm(ArmInterface armInterface) {
    this.armInterface = armInterface;
  }
}
