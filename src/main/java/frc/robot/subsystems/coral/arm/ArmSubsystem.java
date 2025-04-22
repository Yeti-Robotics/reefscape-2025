package frc.robot.subsystems.coral.arm;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.arm.io.ArmStateSetpointIO;
import frc.robot.util.state.StateSubsystem;

public class ArmSubsystem extends StateSubsystem<Angle, ArmPosition, ArmStateSetpointIO> {
    public ArmSubsystem(ArmStateSetpointIO io) {
        super(io);
    }

    public Angle position() {
        return io.getState();
    }
}
