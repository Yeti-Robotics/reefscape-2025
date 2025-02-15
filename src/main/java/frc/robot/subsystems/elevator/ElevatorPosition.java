package frc.robot.subsystems.elevator;

public enum ElevatorPosition {
    BOTTOM(0.0), // placeholder
    LEVEL1(8.0), // placeholder
    LEVEL2(16.0), // placeholder
    LEVEL3(24.0), // placeholder
    LEVEL4(40.0); // placeholder

    private final double height;

    ElevatorPosition(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
