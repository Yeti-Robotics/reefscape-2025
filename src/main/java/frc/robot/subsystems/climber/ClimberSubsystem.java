package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.io.ClimberIO;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO climberIO;

    public ClimberSubsystem(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    private void setClimberSpeed(double speed) {
        climberIO.spinSpeed(speed);
    }

    public Command spinClimber(double speed) {
        return startEnd(() -> setClimberSpeed(speed), climberIO::stop);
    }
}
