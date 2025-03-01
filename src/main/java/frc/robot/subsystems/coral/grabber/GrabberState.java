package frc.robot.subsystems.coral.grabber;

public enum GrabberState {
    ROLL_OUT(GrabberConfig.FORWARD_SPEED),
    ROLL_IN(GrabberConfig.BACKWARD_SPEED),
    OFF(0);

    private final double speed;

    GrabberState(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }
}
