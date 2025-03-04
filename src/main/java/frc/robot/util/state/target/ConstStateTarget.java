package frc.robot.util.state.target;

public class ConstStateTarget<T> implements StateTarget<T> {
    private final T value;

    public ConstStateTarget(T value) {
        this.value = value;
    }

    @Override
    public T get() {
        return null;
    }
}
