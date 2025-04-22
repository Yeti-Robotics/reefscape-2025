package frc.robot.subsystems.coral.grabber;

import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.grabber.io.GrabberSetpointIO;
import frc.robot.util.state.StateSubsystem;

public class GrabberSubsystem extends StateSubsystem<Dimensionless, DimensionlessUnit, GrabberState, GrabberSetpointIO> {
    public GrabberSubsystem(GrabberSetpointIO io) {
        super(io);

        new Trigger(io::hasCoral).onTrue(runOnce(() -> {
        }));
    }
}
