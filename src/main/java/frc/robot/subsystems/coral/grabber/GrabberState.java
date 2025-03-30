package frc.robot.subsystems.coral.grabber;

import frc.robot.util.state.SetpointEnum;

public enum GrabberState implements SetpointEnum<Double> {
    ROLL_OUT(-0.8),
    ROLL_IN(1),
    OFF(0);

    private final double speed;

    GrabberState(double speed) {
        this.speed = speed;
    }

    @Override
    public Double getSetpoint() {
        return speed;
    }
}
