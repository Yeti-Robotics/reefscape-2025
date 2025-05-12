package frc.robot.subsystems.coral.grabber;

import frc.robot.util.state.SetpointProvider;

public enum GrabberState implements SetpointProvider<Double> {
    ROLL_OUT(-0.8),
    ROLL_IN(1),
    OFF(0);

    private final Double speed;

    GrabberState(double speed) {
        this.speed = speed;
    }

    @Override
    public Double getSetpoint() {
        return speed;
    }

    @Override
    public boolean isNear(Double value) {
        return isNear(value, Double.POSITIVE_INFINITY);
    }

    @Override
    public boolean isNear(Double value, Double tolerance) {
        return Math.abs(speed.compareTo(value)) < tolerance;
    }
}
