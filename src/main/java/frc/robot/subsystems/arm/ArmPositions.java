package frc.robot.subsystems.arm;

public enum ArmPositions {
    L1(0),
    L2(0),
    L3(0),
    L4(0.15),
    UP(0.25),
    INTAKE(-0.25);

    private final double value;

    ArmPositions(final double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
}
