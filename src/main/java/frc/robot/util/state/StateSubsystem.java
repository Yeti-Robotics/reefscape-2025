package frc.robot.util.state;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.akit.io.StateSetpointIO;

public abstract class StateSubsystem<T extends Measure<M>, M extends Unit, E extends SetpointEnum<T, M>, H extends StateSetpointIO<E, T, M>>
        extends SubsystemBase {
    private E targetState;
    protected final H io;

    public StateSubsystem(H io) {
        this.io = io;
    }

    public Command transitionTo(E setpoint) {
        targetState = setpoint;
        return runOnce(() -> io.toSetpoint(setpoint));
    }

    public boolean isAt(E setpoint) {
        return io.isAtSetpoint(setpoint);
    }

    public boolean reachedTargetState() {
        return isAt(targetState);
    }

    public T getState() {
        return io.getState();
    }
}
