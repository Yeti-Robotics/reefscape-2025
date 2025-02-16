package frc.robot.subsystems.coral.elevator;

public enum ElevatorPosition {
    // the following positions are placeholders
    // TODO: find actual positions
    BOTTOM(0.0),
    HOVER(0.27),
    L1(8.0),
    L2(16.0),
    L3(24.0),
    L4(40.0);

    private final double height;

    ElevatorPosition(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
