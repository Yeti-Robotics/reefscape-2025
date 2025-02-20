package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ElevatorPosition implements Comparable<ElevatorPosition> {
    // the following positions are placeholders
    // ordering matters!
    // TODO: find actual positions
    BOTTOM(0.0),
    INTAKE(12.0),
    SAFE_POSITION(20.0), // or .27, need to test
    L1(27.0),
    L2(36.0),
    L3(45.0),
    L4(54.0),
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
