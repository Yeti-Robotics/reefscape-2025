package frc.robot.util.state;

public interface StatefulIO<T extends SetpointProvider<V>, V> {
    V getState();

    void setState(V value);

    default void setState(T setpoint) {
        setState(setpoint.getSetpoint());
    }

    default boolean isAtSetpoint(T setpoint) {
        return setpoint.isNear(getState());
    }
}
