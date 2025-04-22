package frc.robot.util.akit.io;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import frc.robot.util.state.SetpointEnum;

public interface MeasuredMotorStateSetpointIO<T extends SetpointEnum<V>, V extends Measure<M>, M extends Unit> extends MotorStateSetpointIO<T, V> {
    @Override
    default boolean isAtSetpoint(V setpoint, V tolerance) {
        return getState().isNear(setpoint, tolerance);
    }
}
