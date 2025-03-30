package frc.robot.subsystems.coral.grabber.io;

import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.util.akit.io.SetpointMotorIO;

public interface GrabberIO extends SetpointMotorIO<GrabberState, Double> {
    boolean hasCoral();
}
