package frc.robot.util.controllerUtils;

public enum POVDirections {
        UP(0),
        DOWN(180),
        LEFT(270),
        RIGHT(90);

        public final int value;

        POVDirections(int value) {
                this.value = value;
        }
}