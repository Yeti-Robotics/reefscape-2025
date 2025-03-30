package frc.robot.util.state;

import edu.wpi.first.wpilibj2.command.Command;

public interface TransitionableSubsystem<T> {
    Command transitionTo(T target);
}
