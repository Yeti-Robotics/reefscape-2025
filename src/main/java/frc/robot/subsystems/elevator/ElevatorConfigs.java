package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;

class ElevatorConfigs {
    static final int primaryElevatorMotorID = 42;
    static final int secondaryElevatorMotorID = 15;
    static final int magSwitchID = 0; // placeholder

    static final double Elevator_kP = 2.4; //placeholder
    static final double Elevator_kI = 0; //placeholder
    static final double Elevator_kD = 0.1; //placeholder

    static Slot0Configs slot0configs = new Slot0Configs()
            .withKP(Elevator_kP)
            .withKI(Elevator_kI)
            .withKD(Elevator_kD);


    public enum ElevatorPosition {
        BOTTOM(0.0), // placeholder
        MIDDLE(24.0), // placeholder
        TOP(40.0); //placeholser

        private final double height;

        ElevatorPosition(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }
}
