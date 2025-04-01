package frc.robot.util.akit.io;

import frc.robot.util.state.SetpointEnum;

public interface PositionalSetpointMotorIO<T extends SetpointEnum<V>, V>
        extends SetpointMotorIO<T, V>, PositionalMotorIO<V> {
    default void toSetpoint(T setpoint) {
        setPosition(setpoint.getSetpoint());
    }
}
