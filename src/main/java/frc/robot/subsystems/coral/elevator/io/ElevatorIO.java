package frc.robot.subsystems.coral.elevator.io;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.state.StateIO;

public interface ElevatorIO extends StateIO<Angle> {
    Trigger zeroTrigger();
}
