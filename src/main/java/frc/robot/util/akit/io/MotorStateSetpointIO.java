package frc.robot.util.akit.io;

import frc.robot.util.state.SetpointEnum;

public interface MotorStateSetpointIO<T extends SetpointEnum<V>, V>
        extends MotorSetpointIO<T, V>, MotorStateIO<V> {
    default void toSetpoint(T setpoint) {
        setState(setpoint.getSetpoint());
    }
}
