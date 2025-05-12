package frc.robot.subsystems.coral.grabber.io;

import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.util.state.StatefulIO;

public interface GrabberIO extends StatefulIO<GrabberState, Double> {
    boolean hasCoral();
}
