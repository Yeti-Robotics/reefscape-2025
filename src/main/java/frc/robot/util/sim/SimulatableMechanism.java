package frc.robot.util.sim;

import edu.wpi.first.units.measure.Angle;

public interface SimulatableMechanism {
    Angle getCurrentPosition();

    Angle getTargetPosition();
}
