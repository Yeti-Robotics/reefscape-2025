package frc.robot.subsystems.coral.grabber;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.grabber.io.GrabberIO;
import frc.robot.util.state.AbstractSetpointStateSubsystem;

public class GrabberSubsystem extends AbstractSetpointStateSubsystem<Double, GrabberState, GrabberIO> {
    public GrabberSubsystem(GrabberIO io) {
        super(io);

        // TODO: add coral trigger
        new Trigger(io::hasCoral).onTrue(runOnce(() -> {}));
    }
}
