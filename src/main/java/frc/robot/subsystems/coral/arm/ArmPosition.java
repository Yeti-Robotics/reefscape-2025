package frc.robot.subsystems.coral.arm;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.state.SetpointEnum;

public enum ArmPosition implements SetpointEnum {
    DOWN(-0.254),
    UP(0.254),
    L1(.60),
    L2(.45),
    L3(.45),
    L4(.30),
    AWAY(0),
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

    @Override
    public void validateOrdering() {

    }
}