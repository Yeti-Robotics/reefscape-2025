package frc.robot.subsystems.coral.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.wrist.io.WristStateSetpointIO;
import frc.robot.util.state.StateSubsystem;

public class WristSubsystem extends StateSubsystem<Angle, WristPosition, WristStateSetpointIO> {
    public WristSubsystem(WristStateSetpointIO io) {
        super(io);
    }

    public Command holdPosition() {
        return runOnce(() -> io.setState(io.getState()));
    }
}
