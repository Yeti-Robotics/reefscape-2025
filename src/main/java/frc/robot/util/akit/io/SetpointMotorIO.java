package frc.robot.util.akit.io;

import frc.robot.util.state.SetpointEnum;

public interface SetpointMotorIO<T extends SetpointEnum<V>, V> extends PositionalMotorIO<V> {
    default void toSetpoint(T setpoint) {
        setPosition(setpoint.getSetpoint());
    };
}
