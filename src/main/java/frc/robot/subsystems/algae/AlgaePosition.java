package frc.robot.subsystems.algae;

public class AlgaePosition {
    public enum Position {
        LOW(0),
        MID(0),
        HIGH(0);

        private final int value;

        Position(final int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }
}
