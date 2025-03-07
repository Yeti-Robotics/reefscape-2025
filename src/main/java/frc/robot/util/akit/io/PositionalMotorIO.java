package frc.robot.util.akit.io;

public interface PositionalMotorIO<T, V> extends InputLoggingIO<T> {
    V getPosition();

    void setPosition(V value);
}
