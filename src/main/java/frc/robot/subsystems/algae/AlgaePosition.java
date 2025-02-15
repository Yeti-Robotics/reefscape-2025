package frc.robot.subsystems.algae;

public enum AlgaePosition {
    LOW(0),
    MID(0),
    HIGH(0);
    private int value;

    AlgaePosition(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
