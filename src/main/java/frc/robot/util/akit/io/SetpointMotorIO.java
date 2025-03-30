package frc.robot.util.akit.io;

import frc.robot.util.state.SetpointEnum;

public interface SetpointMotorIO<T extends SetpointEnum<V>, V> {
    void toSetpoint(T setpoint);

    default boolean isAtSetpoint(T setpoint) {
        return isAtSetpoint(setpoint.getSetpoint());
    }

    default boolean isAtSetpoint(T setpoint, V tolerance) {
        return isAtSetpoint(setpoint.getSetpoint(), tolerance);
    }

    boolean isAtSetpoint(V setpoint);

    boolean isAtSetpoint(V setpoint, V tolerance);
}
