package frc.robot.util.state;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;

public abstract class StatefulSetpointSubsystem<T extends Enum<T>, S extends Unit, M extends Measure<S>, U extends MutableMeasure<S, M, ?>> extends StatefulSubsystem<T> {
    private final U setpointTarget;
    private final M errorTolerance;

    public StatefulSetpointSubsystem(T defaultState, U setPointTarget, M errorTolerance) {
        super(defaultState);
        this.setpointTarget = setPointTarget;
        this.errorTolerance = errorTolerance;
    }

    public abstract StatusSignal<M> currentStateSignal();

    public abstract M determineSetpoint(T targetState);

    public abstract StatusCode moveTo(M setpoint);

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
        return currentStateSignal().getValue().isNear(setpointTarget, errorTolerance);
    }
}
