package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;

class ElevatorConfigs {
    static final int primaryElevatorMotorPort = 42; //placeholder
    static final int secondaryElevatorMotorPort = 0; // placeholder;
    static final int encoderChannelA = 0; //placeholder;
    static final int encoderChannelB = 0; // placeholder
    static final int magSwitchPort = 0; // placeholder

    static final double Elevator_kP = 2.4;
    static final double Elevator_kI = 0;
    static final double Elevator_kD = 0.1;

    static Slot0Configs slot0configs = new Slot0Configs()
            .withKP(Elevator_kP)
            .withKI(Elevator_kI)
            .withKD(Elevator_kD);


    public enum ElevatorPosition {
        BOTTOM(0.0),
        MIDDLE(24.0),
        TOP(48.0);

        private final double height;

        ElevatorPosition(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }
}
