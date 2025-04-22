package frc.robot.util.akit.io;

public interface StateIO<V> {
    V getState();

    void setState(V value);
}
