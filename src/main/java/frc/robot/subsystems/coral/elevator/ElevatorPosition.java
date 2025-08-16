package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ElevatorPosition implements Comparable<ElevatorPosition> {
    BOTTOM(0.0),
    SAFE_POSITION(2.2),
    ALGAE_GROUND(0.85),
    POS_L1(1.9),
    POS_L2(0.43),
    POS_L3(1.74),
    HIGH_ALGAE(2.38),
    SCORE_L3(1.61),
    POS_L4(3.97),
    SCORE_L4(3.97), // 3.4
    LOW_ALGAE(1.175),
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
