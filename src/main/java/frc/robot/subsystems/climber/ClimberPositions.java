package frc.robot.subsystems.climber;

public enum ClimberPositions {
    STOWED(0),
    READY(0.25),
    CLIMBED(0.75);

    private final double value;

    ClimberPositions(final double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
}
