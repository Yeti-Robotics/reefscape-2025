package frc.robot.subsystems.coral.grabber.io;

import frc.robot.util.state.StatefulIO;

public interface GrabberIO extends StatefulIO<Double> {
    boolean hasCoral();
}
