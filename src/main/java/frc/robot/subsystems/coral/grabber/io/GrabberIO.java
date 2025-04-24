package frc.robot.subsystems.coral.grabber.io;

import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.util.akit.io.StateSetpointIO;

public interface GrabberIO extends StateSetpointIO<GrabberState, Double> {
    boolean hasCoral();
}
