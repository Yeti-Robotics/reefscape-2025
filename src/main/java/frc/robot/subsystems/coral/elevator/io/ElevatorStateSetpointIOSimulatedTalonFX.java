package frc.robot.subsystems.coral.elevator.io;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.sim.PhysicsSim;

public class ElevatorStateSetpointIOSimulatedTalonFX implements ElevatorStateSetpointIO {
    private final ElevatorStateSetpointIOTalonFX elevatorIOTalonFX = new ElevatorStateSetpointIOTalonFX();

    public ElevatorStateSetpointIOSimulatedTalonFX() {
        PhysicsSim.getInstance().addTalonFX(elevatorIOTalonFX.primaryElevatorMotor);
    }

    @Override
    public boolean bottomSwitchTriggered() {
        return elevatorIOTalonFX.bottomSwitchTriggered();
    }

    @Override
    public Angle getState() {
        return elevatorIOTalonFX.getState();
    }

    @Override
    public void setState(Angle value) {
        elevatorIOTalonFX.setState(value);
    }
}
