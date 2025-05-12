package frc.robot.util.state;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public interface MeasuredSetpointProvider<V extends Measure<M>, M extends Unit> extends SetpointProvider<V> {
    V getTolerance();

    default boolean isNear(V value) {
        return isNear(value, getTolerance());
    }

    default boolean isNear(V value, V tolerance) {
        return getSetpoint().isNear(value, tolerance);
    }
}
