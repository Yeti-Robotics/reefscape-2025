package frc.robot.subsystems.coral.grabber;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.grabber.io.GrabberIO;
import frc.robot.util.state.StatefulSetpointSubsystem;

public class GrabberSubsystem extends StatefulSetpointSubsystem<Double, GrabberState, GrabberIO> {
    public GrabberSubsystem(GrabberIO io) {
        super(io);

        new Trigger(io::hasCoral).onTrue(runOnce(() -> {
        }));
    }
}
