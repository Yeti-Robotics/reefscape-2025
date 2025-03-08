package frc.robot.util.akit.io;

public interface PositionalMotorIO<V> {
    V getPosition();

    void setPosition(V value);
}
