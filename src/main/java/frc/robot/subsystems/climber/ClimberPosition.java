package frc.robot.subsystems.climber;

public enum ClimberPosition {
    STOWED(0),
    READY(90),
    CLIMBED(270);

    private final int angle;

    ClimberPosition(final int angle) {
        this.angle = angle;
    }

    public int getAngle() {
        return angle;
    }
}
