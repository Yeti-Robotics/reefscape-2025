package frc.robot.util.akit.io;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import frc.robot.util.state.SetpointEnum;

public interface StateSetpointIO<T extends SetpointEnum<V, M>, V extends Measure<M>, M extends Unit>
        extends StateIO<V> {
    default void toSetpoint(T setpoint) {
        setState(setpoint.getSetpoint());
    }

    default boolean isAtSetpoint(T setpoint) {
        return isAtSetpoint(setpoint, setpoint.getTolerance());
    }

    default boolean isAtSetpoint(T setpoint, V tolerance) {
        return setpoint.getSetpoint().isNear(getState(), tolerance);
    }
}
