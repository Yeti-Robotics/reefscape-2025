package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LEDStrip extends SubsystemBase {

    public enum State {
        IDLE,
        CORALINTAKE,
        CORALSTOW,
        ALGAEINTAKE,
        NICK;
    }

    private final LEDStripIO io;
    private State state = State.IDLE;

    public LEDStrip(LEDStripIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            // progress bar
        }

        switch (state) {
            case IDLE:
                allianceAnim();
                break;
            case CORALINTAKE:
                io.coralIntakeAnimation();
                break;
            case CORALSTOW:
                io.coralStowAnimation();
                break;
            case ALGAEINTAKE:
                io.algaeIntakeAnimation();
                break;
            case NICK:
                io.nick();
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    private void allianceAnim() {
        if (Robot.isRedAlliance()) {
            io.idleRedAlliance();
        } else {
            io.idleBlueAlliance();
        }
    }

    public Command setStateCommand(State state) {
        return runOnce(() -> setState(state));
    }
}
