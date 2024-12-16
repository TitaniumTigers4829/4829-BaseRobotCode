package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class SimulatedElevator implements ElevatorInterface {
    private ElevatorSim simulatedElevator = new ElevatorSim(DCMotor.getFalcon500(2), ElevatorConstants.ELEVATOR_GEAR_RATIO, ElevatorConstants.ELEVATOR_CARRIAGE_MASS, ElevatorConstants.DRUM_RADIUS, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT, true, 0.0);
    private PIDController simPID;
    private double currentVolts;

    public SimulatedElevator() {
        simPID = new PIDController(ElevatorConstants.ELEVATOR_P, ElevatorConstants.ELEVATOR_I, ElevatorConstants.ELEVATOR_D);
    }

    public void updateInputs(ElevatorInputs inputs) {
        inputs.leaderMotorPosition = getElevatorPosition();
        inputs.followerMotorPosition = getElevatorPosition();
    }

    public void setElevatorPosition(double position) {
        simulatedElevator.setState(simPID.calculate(position), ElevatorConstants.VELOCITY_METERS_PER_SECOND);
    }

    public double getElevatorPosition() {
        return simulatedElevator.getPositionMeters();
    }

    public void setVolts(double volts) {
        currentVolts = simPID.calculate(volts);
        simulatedElevator.setInputVoltage(currentVolts);
    }

    public double getVolts() {
        return currentVolts;
    }
}
