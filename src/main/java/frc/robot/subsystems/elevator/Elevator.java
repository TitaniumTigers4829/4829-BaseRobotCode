package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private ElevatorInterface elevatorInterface;
  private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  public Elevator(ElevatorInterface elevatorInterface) {
    this.elevatorInterface = elevatorInterface;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
