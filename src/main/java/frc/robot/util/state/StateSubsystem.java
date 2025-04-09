package frc.robot.util.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.akit.io.SetpointMotorIO;

public abstract class StateSubsystem<T, E extends SetpointEnum<T>, H extends SetpointMotorIO<E, T>>
        extends SubsystemBase implements TransitionableSubsystem<E> {
    private E targetState;
    protected final H io;

    public StateSubsystem(H io) {
        this.io = io;
    }

    @Override
    public Command transitionTo(E setpoint) {
        targetState = setpoint;
        return runOnce(() -> io.toSetpoint(setpoint));
    }

    public boolean isAt(E setpoint) {
        return io.isAtSetpoint(setpoint);
    }

    public E getTargetState() {
        if (isAt(targetState)) {
            targetState = null;
        }

        return targetState;
    }
}
