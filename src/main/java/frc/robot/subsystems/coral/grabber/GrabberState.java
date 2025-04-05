package frc.robot.subsystems.coral.grabber;

public enum GrabberState {
    ROLL_OUT(GrabberConfig.OUTSPIT),
    ROLL_IN(GrabberConfig.INTAKE),
    HOLD(GrabberConfig.HOLD),
    OFF(0);

    private final double speed;

    GrabberState(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }
}
