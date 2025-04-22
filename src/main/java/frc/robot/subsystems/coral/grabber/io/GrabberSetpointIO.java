package frc.robot.subsystems.coral.grabber.io;

import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.util.akit.io.StateSetpointIO;


public interface GrabberSetpointIO extends StateSetpointIO<GrabberState, Dimensionless, DimensionlessUnit> {
    boolean hasCoral();
}
