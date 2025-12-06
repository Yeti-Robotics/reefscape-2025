package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double climberPosition = 0.0;
        public double climberVelocityRPM = 0.0;
    }

    public default void updateInputs(ClimberIO.ClimberIOInputs inputs) {}

    public default void setClimberSpeed(double speed) {}

    public default void stop() {}

    public default void setClimberPosition(ClimberPosition position) {}
}
