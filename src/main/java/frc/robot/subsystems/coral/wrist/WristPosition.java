package frc.robot.subsystems.coral.wrist;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.wrist.io.WristConfigs;
import frc.robot.util.state.SetpointEnum;

public enum WristPosition implements SetpointEnum<Angle, AngleUnit> {
    SAFE(0),
    UNSAFE(0.25);

    private final Angle angle;

    WristPosition(double angle) {
        this(Units.Rotations.of(angle));
    }

    WristPosition(Angle angle) {
        this.angle = angle;
    }

    @Override
    public Angle getSetpoint() {
        return angle;
    }

    @Override
    public Angle getTolerance() {
        return WristConfigs.WRIST_TOLERANCE;
    }
}
