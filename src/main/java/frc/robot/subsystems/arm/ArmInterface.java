package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmInterface {
  
  @AutoLog
  public static class ArmInputs {
    public double angle = 0.0;
  }

  public default void updateInputs(ArmInputs inputs) {}
  
  public default void setAngle(double angle) {}

  public default double getAngle() {
    return 0.0;
  }
}
 