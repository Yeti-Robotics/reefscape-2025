package frc.robot.util.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractStateSubsystem<T, E extends SetpointProvider<T>, H extends StatefulIO<E, T>>
        extends SubsystemBase {
    private E targetState;
    protected final H io;

    public AbstractStateSubsystem(H io) {
        this.io = io;
    }

    public Command transitionTo(E setpoint) {
        targetState = setpoint;
        return transitionAsync(setpoint).andThen(Commands.waitUntil(this::reachedTargetState));
    }

    public Command transitionAsync(E setpoint) {
        targetState = setpoint;
        return runOnce(() -> io.setState(setpoint));
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
