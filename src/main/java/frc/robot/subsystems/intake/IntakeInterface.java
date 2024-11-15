package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeInterface {
  @AutoLog
  public static class IntakeInputs {
    public boolean isConnected = true;

    // intake motor
    public double intakeVelocity = 0.0;
    public double intakeTemp = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;

    // pivot
    public double pivotPosition = 0.0;
    public double pivotVelocity = 0.0;
    public double pivotTemp = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeInputs inputs) {}

  default void setPivotPosition(double position) {}

  default void setIntakeSpeed(double speed) {}

  default double getIntakeSpeed() {
    return 0.0;
  }

  default void setPivotSpeed(double speed) {}

  default double getPivotPosition() {
    return 0.0;
  }
}