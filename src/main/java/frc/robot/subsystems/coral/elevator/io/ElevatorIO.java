package frc.robot.subsystems.coral.elevator.io;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.util.akit.io.MotorControlIO;
import frc.robot.util.akit.io.PositionalSetpointMotorIO;
import frc.robot.util.akit.io.SetpointMotorIO;

public interface ElevatorIO
        extends PositionalSetpointMotorIO<ElevatorPosition, Angle>, MotorControlIO<ControlRequest> {
    boolean bottomSwitchTriggered();
}
