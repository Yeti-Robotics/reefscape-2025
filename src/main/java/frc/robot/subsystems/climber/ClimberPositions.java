package frc.robot.subsystems.climber;

public enum ClimberPositions {
    STOWED(0),
    READY(90),
    CLIMBED(270);

    private final int angle;

    ClimberPositions(final int angle) { this.angle = angle; }

    public int getAngle() {
        return angle;
    }
}
