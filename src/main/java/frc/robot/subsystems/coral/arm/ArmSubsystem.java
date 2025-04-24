package frc.robot.subsystems.coral.arm;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.arm.io.ArmIO;
import frc.robot.util.state.StateSubsystem;

public class ArmSubsystem extends StateSubsystem<Angle, ArmPosition, ArmIO> {
    public ArmSubsystem(ArmIO io) {
        super(io);
    }
}
