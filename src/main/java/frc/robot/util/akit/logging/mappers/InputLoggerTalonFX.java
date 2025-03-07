package frc.robot.util.akit.logging.mappers;

import com.ctre.phoenix6.StatusSignal;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class InputLoggerTalonFX<T> extends InputLogger<T> {
    private boolean isDelegate = false;


    public <V> InputLoggerTalonFX(Map<FieldSetter<T, V>, StatusSignal<V>> mappings) {
        super(populateSignals(mappings));
    }


    private <V> Map<FieldSetter<T, V>, Supplier<V>> populateSignals(
            Map<FieldSetter<T, V>, StatusSignal<V>> mappings
    ) {

    }


    @Override
    public void updateInputs(T inputs) {
        inputSetters.forEach(i -> i.accept(inputs));
    }
}
