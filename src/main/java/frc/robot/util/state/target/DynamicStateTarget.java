package frc.robot.util.state.target;

import java.util.function.Supplier;

public class DynamicStateTarget<T> implements StateTarget<T> {
    private final Supplier<T> valueProvider;

    public DynamicStateTarget(Supplier<T> valueProvider) {
        this.valueProvider = valueProvider;
    }

    @Override
    public T get() {
        return valueProvider.get();
    }
}
