package frc.robot.util.state;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;

public class StateUtils {
    public static MutAngle mutableAngleSetpoint(AngleUnit unit) {
        return new MutAngle(0,0, unit);
    }

    public static MutAngle mutableRotationSetpoint() {
        return new MutAngle(0, 0, Units.Rotations);
    }
}
