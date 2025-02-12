package frc.robot.subsystems.elevator;

public enum ElevatorPosition {
    BOTTOM(0.0), // placeholder
    LEVEL1(0.0), // placeholder
    LEVEL2(0.0), // placeholder
    LEVEL3(0.0), // placeholder
    LEVEL4(0.0); // placeholder

    private final double height;

    ElevatorPosition(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
