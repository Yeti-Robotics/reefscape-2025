package frc.robot.util.state;

public interface StatefulIO<V> {
    V getState();

    void setState(V value);
}
