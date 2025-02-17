package frc.robot.util.state;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;

public abstract class StatefulSetpointSubsystem<T extends Enum<T>, S extends Unit, M extends Measure<S>> extends StatefulSubsystem<T> {
    private MutableMeasure<S, Measure<S>, ?> setpointTarget;

    public StatefulSetpointSubsystem(T defaultState) {
        super(defaultState);
    }

    public abstract StatusSignal<M> currentStateSignal();

    public abstract M determineSetpoint(T targetState);

    public abstract StatusCode moveTo(M setpoint);

    public abstract M getErrorTolerance();

    @Override
    protected StatusCode initializeTransition(T targetState) {
        M setPointMeasure = determineSetpoint(targetState);
        setpointTarget.mut_replace(setPointMeasure);
        return moveTo(setPointMeasure);
    }

    @Override
    public void runPeriodic() {
        currentStateSignal().refresh();
    }

    @Override
    protected boolean isTransitionFinished() {
        return currentStateSignal().getValue().isNear(setpointTarget, getErrorTolerance());
    }
}
