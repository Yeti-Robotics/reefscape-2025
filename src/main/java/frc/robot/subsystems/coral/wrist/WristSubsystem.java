package frc.robot.subsystems.coral.wrist;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.wrist.io.WristIO;
import frc.robot.util.state.StatefulSetpointSubsystem;

public class WristSubsystem extends StatefulSetpointSubsystem<Angle, WristPosition, WristIO> {
    public WristSubsystem(WristIO io) {
        super(io);
    }
}
