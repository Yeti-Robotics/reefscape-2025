package frc.robot.subsystems.elevator;

import frc.robot.subsystems.Superstructure;

import static frc.robot.subsystems.Superstructure.SuperState.*;
public class ElevatorStateMachine {
    private Superstructure.SuperState superState;
    private ElevatorPosition currentPosition;

    public ElevatorStateMachine() {
        this.superState = IDLE;
        this.currentPosition = ElevatorPosition.BOTTOM;
    }

    public void moveToPosition(ElevatorPosition targetPosition) {
        if (targetPosition != currentPosition) {
            update(targetPosition);
            currentPosition = targetPosition;
        }
    }

    public Superstructure.SuperState update(ElevatorPosition targetPosition) {
        return switch (targetPosition) {
            case LEVEL1 -> PREPARE_L1;
            case LEVEL2 -> PREPARE_L1;
            case LEVEL3 -> PREPARE_L1;
            case LEVEL4 -> PREPARE_L1;
            default -> IDLE;
        };
    }
    public Superstructure.SuperState getSuperState() {
        return superState;
    }
    public ElevatorPosition getCurrentPosition() {
        return currentPosition;
    }
}