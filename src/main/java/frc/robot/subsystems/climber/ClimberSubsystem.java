package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.io.ClimberConfig;
import frc.robot.subsystems.climber.io.ClimberIO;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO climberIO;

    public ClimberSubsystem(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    public Command spinClimber(double speed) {
        return startEnd(() -> climberIO.spinSpeed(speed), climberIO::stop);
    }

    public Command climbUp() {
        return spinClimber(ClimberConfig.CLIMB_SPEED);
    }

    public Command climbDown() {
        return spinClimber(ClimberConfig.UNCLIMB_SPEED);
    }
}
