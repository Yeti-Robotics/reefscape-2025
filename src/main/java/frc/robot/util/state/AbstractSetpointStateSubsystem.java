package frc.robot.util.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractSetpointStateSubsystem<T, E extends SetpointProvider<T>, H extends StatefulIO<T>>
        extends SubsystemBase {
    private E targetState;
    protected final H io;

    public AbstractSetpointStateSubsystem(H io) {
        this.io = io;
    }

    public Command transitionTo(E setpoint) {
        targetState = setpoint;
        return transitionAsync(setpoint).andThen(Commands.waitUntil(this::reachedTargetState));
    }

    public Command transitionAsync(E setpoint) {
        targetState = setpoint;
        return runOnce(() -> io.setState(setpoint.getSetpoint()));
    }

    public boolean isAt(E setpoint) {
        return setpoint.isNear(io.getState());
    }

    public boolean isNear(E setpoint, T tolerance) {
        return setpoint.isNear(setpoint.getSetpoint(), tolerance);
    }

    public boolean reachedTargetState() {
        return isAt(targetState);
    }

    public T getState() {
        return io.getState();
    }
}
