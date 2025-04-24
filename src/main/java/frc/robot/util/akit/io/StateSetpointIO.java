package frc.robot.util.akit.io;

import frc.robot.util.state.SetpointEnum;

public interface StateSetpointIO<T extends SetpointEnum<V>, V> extends StateIO<V> {
    default void toSetpoint(T setpoint) {
        setState(setpoint.getSetpoint());
    }

    default boolean isAtSetpoint(T setpoint) {
        return setpoint.isNear(getState());
    }

    default boolean isAtSetpoint(T setpoint, V tolerance) {
        return setpoint.isNear(getState(), tolerance);
    }
}
