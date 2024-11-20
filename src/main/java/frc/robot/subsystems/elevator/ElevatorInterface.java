package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorInterface {
  
  @AutoLog
  public static class ElevatorInputs {
    public double leaderMotorPosition = 0.0;
    public double leaderMotorVelocity = 0.0;
    public double leaderMotorAppliedVolts = 0.0;
    public double leaderMotorCurrentAmps = 0.0;

    public double followerMotorPosition = 0.0;
    public double followerMotorVelocity = 0.0;
    public double followerMotorAppliedVolts = 0.0;
    public double followerMotorCurrentAmps = 0.0;
  }

  public default void updateInputs(ElevatorInputs inputs) {}

  public default double getElevatorPosition() {
    return 0.0;
  }

  public default void setElevatorPosition(double position) {}

  public default void setElevatorSpeed(double speed) {}

  public default void setVolts(double volts) {}

  public default double getVolts() {
    return 0.0;
  }
}
