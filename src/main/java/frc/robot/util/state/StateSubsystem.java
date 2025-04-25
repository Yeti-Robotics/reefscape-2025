package frc.robot.util.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.akit.io.StateSetpointIO;

public abstract class StateSubsystem<T, E extends SetpointEnum<T>, H extends StateSetpointIO<E, T>>
        extends SubsystemBase {
    private E targetState;
    protected final H io;

    public StateSubsystem(H io) {
        this.io = io;
    }

    public Command transitionTo(E setpoint) {
        targetState = setpoint;
        return runOnce(() -> io.toSetpoint(setpoint))
                .andThen(Commands.waitUntil(this::reachedTargetState))
                .andThen(Robot.isSimulation() ? Commands.waitSeconds(0.5) : Commands.none());
    }

    public boolean isAt(E setpoint) {
        return io.isAtSetpoint(setpoint);
    }

    public boolean reachedTargetState() {
        System.out.println(io.getState() + " " + targetState);
        return isAt(targetState);
    }

    public T getState() {
        return io.getState();
    }
}
