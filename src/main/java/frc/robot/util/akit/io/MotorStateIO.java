package frc.robot.util.akit.io;

public interface MotorStateIO<V> {
    V getState();

    void setState(V value);
}
