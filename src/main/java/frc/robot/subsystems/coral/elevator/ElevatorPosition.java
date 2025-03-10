package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ElevatorPosition implements Comparable<ElevatorPosition> {
    BOTTOM(0.0),
    INTAKE(1.9),
    SAFE_POSITION(2.2),
    POS_L1(2.4),
    POS_L2(0.367920),
    POS_L3(1.8),
    POS_L4(3.95),
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
