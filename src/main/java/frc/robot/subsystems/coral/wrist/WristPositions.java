package frc.robot.subsystems.coral.wrist;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum WristPositions {
    SAFE(0),
    UNSAFE(0.25),
    INTAKE(-0.25),
    HOLD(-1);

    private final Angle angle;

    WristPositions(double angle) {
        this.angle = Units.Rotations.of(angle);
    }

    public Angle getAngle() {
        return angle;
    }
}
