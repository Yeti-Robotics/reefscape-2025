package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double positionRotation = 0.0;
        public double velocityRPM = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    // other methods for the motors
    // they only need to be things that the motors to do bc tou build the whole subsystem in subsystem
    // for example:
    public default void moveTo(Angle pos) {}

    public default void zeroPos() {}
}
