package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    public Command spinClimber(double speed) {
        return runEnd(() -> io.setClimberSpeed(speed), () -> io.stop());
    }

    public void target(ClimberPosition position) {
        io.setClimberPosition(position);
    }
}
