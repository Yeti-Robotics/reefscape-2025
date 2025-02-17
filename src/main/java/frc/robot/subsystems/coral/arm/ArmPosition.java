package frc.robot.subsystems.coral.arm;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ArmPosition {
    STOWED(90),
    L1(0),
    L2(0),
    L3(0),
    L4(0),
    HOLD(-1);  // special case, for when transitions are interrupted

    private final Angle angle;

    ArmPosition(double angle) {
        this(Units.Rotations.of(angle));
    }

    ArmPosition(Angle angle) {
        this.angle = angle;
    }

    public Angle getAngle() {
        return angle;
    }
}