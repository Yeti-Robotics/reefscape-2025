package frc.robot.util.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public abstract class AbstractStateSubsystem<T, H extends StatefulIO<T>> {
    protected final H io;

    public AbstractStateSubsystem(H io) {
        this.io = io;
    }

    public Command transitionTo(T setpoint) {
        return transitionAsync(setpoint).andThen(Commands.waitUntil(() -> isAt(setpoint)));
    }

    public Command transitionAsync(T setpoint) {
        return runOnce(() -> io.setState(setpoint));
    }

    public abstract boolean isAt(T setpoint);

    public T getState() {
        return io.getState();
    }
}
