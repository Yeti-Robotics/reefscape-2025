package frc.robot.util.state;

public interface SetpointEnum<V> {
    V getSetpoint();

    boolean isNear(V value);

    boolean isNear(V value, V tolerance);
}
