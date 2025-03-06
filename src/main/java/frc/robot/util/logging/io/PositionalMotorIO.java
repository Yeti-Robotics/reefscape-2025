package frc.robot.util.logging.io;

public interface PositionalMotorIO<T, V> extends InputLoggingIO<T> {
    V getPosition();
    void setPosition(V value);
}
