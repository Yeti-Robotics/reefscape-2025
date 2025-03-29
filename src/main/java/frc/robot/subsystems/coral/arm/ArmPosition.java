package frc.robot.subsystems.coral.arm;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.state.SetpointEnum;

public enum ArmPosition implements SetpointEnum {
    DOWN(-0.254),
    UP(0.254),
    POS_L1(-0.1),
    POS_L2(.16),
    SCORE_L2(.07),
    POS_L3(.13),
    SCORE_L3(.06),
    POS_L4(.15),
    SCORE_L4(.05),
    AWAY(0),
    GROUND(-0.029785),
    HP(0.190918),
    AWAY_BUMPER(0.05),
    CLIMB_L4(0.35),
    SCORE_CLIMB_L4(0.45),
    CLIMB_L3(0.37),
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

    @Override
    public void validateOrdering() {}
}
