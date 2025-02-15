package frc.robot.subsystems.arm;

import frc.robot.subsystems.Superstructure;

public class ArmStateMachine {
    private ArmPositions currentState = ArmPositions.IDLE;
    static Superstructure.SuperState superState;

    public void setState(ArmPositions newState) {
        currentState = newState;
    }

    public ArmPositions getCurrentState() {
        return currentState;
    }

    public void moveToPosition(ArmPositions targetPosition) {
        if (targetPosition != currentState) {
            update(targetPosition);
            currentState = targetPosition;
        }
    }

    public Superstructure.SuperState update(ArmPositions targetPosition) {
        switch (currentState) {
            case IDLE:
                superState = Superstructure.SuperState.IDLE;
                break;
            case STOWED:
                superState = Superstructure.SuperState.INTAKE_CORAL;
                break;
            case L1:
                superState = Superstructure.SuperState.PREPARE_L1;
                break;
            case L2:
                superState = Superstructure.SuperState.PREPARE_L2;
                break;
            case L3:
                superState = Superstructure.SuperState.PREPARE_L3;
                break;
            case L4:
                superState = Superstructure.SuperState.PREPARE_L4;
                break;
        }
        return superState;
    }

    public static Superstructure.SuperState getSuperState() {
        return superState;
    }
}
