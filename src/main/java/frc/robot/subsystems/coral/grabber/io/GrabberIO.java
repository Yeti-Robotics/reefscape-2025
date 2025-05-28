package frc.robot.subsystems.coral.grabber.io;

import frc.robot.util.state.StateIO;

public interface GrabberIO extends StateIO<Double> {
    boolean hasCoral();
}
