package frc.robot.subsystems.arm;

public enum ArmPositions {
    STOWED(90);

    private final double angle;

    ArmPositions(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}