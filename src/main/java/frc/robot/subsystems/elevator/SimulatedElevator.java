package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class SimulatedElevator implements ElevatorInterface {
    private ElevatorSim simulatedElevator = new ElevatorSim(DCMotor.getFalcon500(2), );

    public SimulatedElevator() {

    }
}
