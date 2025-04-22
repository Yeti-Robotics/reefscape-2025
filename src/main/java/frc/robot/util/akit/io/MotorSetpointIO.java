package frc.robot.util.akit.io;

import frc.robot.util.state.SetpointEnum;

public interface MotorSetpointIO<T extends SetpointEnum<V>, V> {
    void toSetpoint(T setpoint);

    V getTolerance();

    default boolean isAtSetpoint(T setpoint) {
        return isAtSetpoint(setpoint.getSetpoint(), getTolerance());
    }

    default boolean isAtSetpoint(T setpoint, V tolerance) {
        return isAtSetpoint(setpoint.getSetpoint(), tolerance);
    }

    default boolean isAtSetpoint(V setpoint) {
        return isAtSetpoint(setpoint, getTolerance());
    }

    boolean isAtSetpoint(V setpoint, V tolerance);
}
