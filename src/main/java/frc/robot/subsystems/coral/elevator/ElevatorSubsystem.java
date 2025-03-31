package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.elevator.io.ElevatorIO;
import frc.robot.subsystems.coral.elevator.io.ElevatorInputs;
import frc.robot.util.akit.io.InputLoggingIO;
import frc.robot.util.state.StatefulSetpointSubsystem;

public class ElevatorSubsystem extends StatefulSetpointSubsystem<Angle, ElevatorPosition, ElevatorIO> implements InputLoggingIO<ElevatorInputs> {
    public ElevatorSubsystem(ElevatorIO io) {
        super(io);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.targetSetpoint = getTargetState();
    }

    public Angle position() {
        return io.getPosition();
    }
}
