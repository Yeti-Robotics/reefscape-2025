package frc.robot.util.akit.io.impl;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.akit.io.PositionalMotorIO;

public abstract class PositionalMotorIOTalonFX<T, V extends Angle>
        implements PositionalMotorIO<T, V> {

    @Override
    public V getPosition() {
        return null;
    }

    @Override
    public void setPosition(V value) {}

    public abstract void updateInputs(T inputs);
}
