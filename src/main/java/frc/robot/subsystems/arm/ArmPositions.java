package frc.robot.subsystems.arm;

public enum ArmPositions {
    STOWED(0.25),
    INTAKE(0.75);

    private final double value;

    ArmPositions(final double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
}
