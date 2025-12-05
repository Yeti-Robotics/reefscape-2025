package frc.robot.util;

import edu.wpi.first.units.measure.Angle;

public interface SimulatableMechanism {
    Angle getCurrentPosition();

    Angle getTargetPosition();
}
