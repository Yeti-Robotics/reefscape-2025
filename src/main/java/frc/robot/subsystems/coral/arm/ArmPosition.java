package frc.robot.subsystems.coral.arm;

public enum ArmPosition {
    HOLD(0),
    STOWED(90),
    L1(0),
    L2(0),
    L3(0),
    L4(0);

    private final double angle;

    ArmPosition(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}