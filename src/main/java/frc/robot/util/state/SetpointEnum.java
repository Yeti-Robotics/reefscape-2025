package frc.robot.util.state;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public interface SetpointEnum<V extends Measure<M>, M extends Unit> {
    V getSetpoint();

    V getTolerance();

    default boolean isNear(V value) {
        return isNear(value, getTolerance());
    }

    default boolean isNear(V value, V tolerance) {
        return getSetpoint().isNear(value, tolerance);
    }
}
