package frc.robot.subsystems.coral.grabber.io.sensor;

import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public interface CoralSensor {
    boolean hasCoral();

    static CoralSensor createCoralSensor() {
        return Robot.isSimulation() && !Logger.hasReplaySource() ? new SimulatedCoralSensor() : new CANColorCoralSensor();
    }
}
