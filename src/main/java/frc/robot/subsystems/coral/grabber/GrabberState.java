package frc.robot.subsystems.coral.grabber;

public enum GrabberState {
    ROLL_OUT(GrabberConfig.OUTSPIT),
    ROLL_IN(GrabberConfig.INTAKE),
    HOLD(GrabberConfig.HOLD),
    ALGAE_HOLD(GrabberConfig.ALGAE_HOLD),
    ALGAE_ROLL_IN(GrabberConfig.ALGAE_INTAKE),
    ALGAE_SHOOT(GrabberConfig.ALGAE_SHOOT),
    ALL_IN(GrabberConfig.ALL_IN),
    OFF(0);

    private final double speed;

    GrabberState(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }
}
