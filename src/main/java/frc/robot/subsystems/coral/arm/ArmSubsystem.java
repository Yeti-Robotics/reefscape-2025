package frc.robot.subsystems.coral.arm;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.arm.io.ArmIO;
import frc.robot.subsystems.coral.arm.io.ArmInputs;
import frc.robot.util.akit.io.InputLoggingIO;
import frc.robot.util.state.StateSubsystem;

public class ArmSubsystem extends StateSubsystem<Angle, ArmPosition, ArmIO>
        implements InputLoggingIO<ArmInputs> {
    public ArmSubsystem(ArmIO io) {
        super(io);
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.targetSetpoint = getTargetState();
    }

    public Angle position() {
        return io.getPosition();
    }
}
