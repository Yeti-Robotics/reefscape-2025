package frc.robot.subsystems.coral.grabber.io;

import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.util.akit.io.MotorSetpointIO;

public interface GrabberSetpointIO extends MotorSetpointIO<GrabberState, Double> {
    boolean hasCoral();
}
