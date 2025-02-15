package frc.robot.subsystems.elevator;

import frc.robot.subsystems.Superstructure;
public class ElevatorStateMachine {
    private final Superstructure.SuperState superState;
    private final ElevatorPosition currentPosition;
    public ElevatorStateMachine() {
        this.superState = Superstructure.SuperState.IDLE;
        this.currentPosition = ElevatorPosition.BOTTOM;
    }
}