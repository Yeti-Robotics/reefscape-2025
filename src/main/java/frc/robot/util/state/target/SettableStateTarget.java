package frc.robot.util.state.target;

import java.util.function.Supplier;

public class SettableStateTarget<T> implements StateTarget<T> {
    private final Supplier<T> valueProvider;

    public SettableStateTarget(Supplier<T> valueProvider) {
        this.valueProvider = valueProvider;
    }

    @Override
    public T get() {
        return valueProvider.get();
    }
}
