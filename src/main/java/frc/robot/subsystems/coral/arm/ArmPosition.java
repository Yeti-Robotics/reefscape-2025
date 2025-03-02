package frc.robot.subsystems.coral.arm;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ArmPosition {
    DOWN(-0.254),
    UP(0.254),
    POS_L1(-0.1),
    POS_L2(.13),
    SCORE_L2(.07),
    POS_L3(.13),
    SCORE_L3(.06),
    POS_L4(.15),
    SCORE_L4(.05),
    AWAY(0),
    HOLD(-1); // special case, for when transitions are interrupted

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
