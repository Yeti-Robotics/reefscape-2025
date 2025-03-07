package frc.robot.util.akit.logging.mappers;

import frc.robot.util.akit.io.InputLoggingIO;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public abstract class InputLogger<T> implements InputLoggingIO<T> {
    private final List<Consumer<T>> inputSetters = new ArrayList<>();

    protected <V> InputLogger(Map<FieldSetter<T, V>, Supplier<V>> mappings) {
        for (var entry : mappings.entrySet()) {
            inputSetters.add((t) -> entry.getKey().set(t, entry.getValue().get()));
        }
    }

    @Override
    public void updateInputs(T inputs) {
        inputSetters.forEach(i -> i.accept(inputs));
    }
}
