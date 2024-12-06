package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private ElevatorInterface elevatorInterface;
  private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  public Elevator(ElevatorInterface elevatorInterface) {
    this.elevatorInterface = elevatorInterface;
  }

  public double getElevatorPosition() {
    return elevatorInterface.getElevatorPosition();
  }

  public double getVolts(){
    return elevatorInterface.getVolts();
  }

  public void setElevatorPosition(double position){
    elevatorInterface.setElevatorPosition(position);
  }

  public void setElevatorSpeed(double speed){
    elevatorInterface.setElevatorSpeed(speed);
  }

  public void setVolts(double volts){
    elevatorInterface.setVolts(volts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
