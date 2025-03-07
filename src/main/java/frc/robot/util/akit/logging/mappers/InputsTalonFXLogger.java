package frc.robot.util.akit.logging.mappers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.util.akit.io.InputLoggingIO;

public abstract class InputsTalonFXLogger<T> implements InputLoggingIO<T> {
    private final boolean shouldRefresh;

    public InputsTalonFXLogger(final boolean shouldRefresh) {
        this.shouldRefresh = shouldRefresh;
    }

    protected abstract StatusSignal<?>[] statusSignal();
    protected abstract void talonInputs(T t);

    public void refreshSignals(BaseStatusSignal... signals) {
        BaseStatusSignal.refreshAll(signals);
    }

    @Override
    final public void updateInputs(T inputs) {
        if (shouldRefresh) {
            refreshSignals(statusSignal());
        }

        talonInputs(inputs);
    }
}
