package frc.robot.subsystems.arm;

public enum ArmPositions {
    IDLE(-1),
    STOWED(90),
    L1(0),
    L2(0),
    L3(0),
    L4(0);

    private final double angle;

    ArmPositions(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}