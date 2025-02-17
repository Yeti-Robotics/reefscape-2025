package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ElevatorPosition {
    // the following positions are placeholders
    // TODO: find actual positions
    BOTTOM(0.0),
    HOVER(0.27),
    L1(8.0),
    L2(16.0),
    L3(24.0),
    L4(40.0),
    HOLD(-1); // special case, for when transitions are interrupted

    private final Angle height;

    ElevatorPosition(double height) {
        this(Units.Rotations.of(height));
    }

    ElevatorPosition(Angle height) {
        this.height = height;
    }

    public Angle getHeight() {
        return height;
    }
}
