package frc.robot.subsystems.coral.elevator.io;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.sim.PhysicsSim;

public class ElevatorIOSimulatedTalonFX implements ElevatorIO {
    private final ElevatorIOTalonFX elevatorIOTalonFX = new ElevatorIOTalonFX();

    public ElevatorIOSimulatedTalonFX() {
        PhysicsSim.getInstance().addTalonFX(elevatorIOTalonFX.primaryElevatorMotor);
    }

    @Override
    public boolean bottomSwitchTriggered() {
        return elevatorIOTalonFX.bottomSwitchTriggered();
    }

    @Override
    public void setCurrentPositionToZero() {
        elevatorIOTalonFX.setCurrentPositionToZero();
    }

    @Override
    public void stopOutput() {
        elevatorIOTalonFX.stopOutput();
    }

    @Override
    public Angle getPosition() {
        return elevatorIOTalonFX.getPosition();
    }

    @Override
    public void setPosition(Angle value) {
        elevatorIOTalonFX.setPosition(value);
    }

    @Override
    public boolean isAtSetpoint(Angle setpoint) {
        return elevatorIOTalonFX.isAtSetpoint(setpoint);
    }

    @Override
    public boolean isAtSetpoint(Angle setpoint, Angle tolerance) {
        return elevatorIOTalonFX.isAtSetpoint(setpoint, tolerance);
    }
}
