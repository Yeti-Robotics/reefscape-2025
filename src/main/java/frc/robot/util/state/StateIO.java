package frc.robot.util.state;

public interface StateIO<V> {
    V getState();

    void setState(V value);
}
