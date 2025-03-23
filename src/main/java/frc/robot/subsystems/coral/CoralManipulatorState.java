package frc.robot.subsystems.coral;

import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.grabber.GrabberState;

public enum CoralManipulatorState {
    INTAKE_CORAL(ArmPosition.DOWN, ElevatorPosition.INTAKE, GrabberState.ROLL_IN),
    L1(ArmPosition.POS_L1, ElevatorPosition.POS_L1, GrabberState.OFF),
    SCORE_L1(ArmPosition.SCORE_L1, ElevatorPosition.POS_L1, GrabberState.ROLL_OUT),
    L2(ArmPosition.POS_L2, ElevatorPosition.POS_L2, GrabberState.OFF),
    SCORE_L2(ArmPosition.SCORE_L2, ElevatorPosition.POS_L2, GrabberState.ROLL_OUT),
    L3(ArmPosition.POS_L3, ElevatorPosition.SAFE_POSITION, GrabberState.OFF),
    SCORE_L3(ArmPosition.SCORE_L3, ElevatorPosition.POS_L3, GrabberState.ROLL_OUT),
    L4(ArmPosition.POS_L4, ElevatorPosition.POS_L4, GrabberState.OFF),
    SCORE_L4(ArmPosition.SCORE_L4, ElevatorPosition.POS_L4, GrabberState.ROLL_OUT),
    SCOREREEF(ArmPosition.DOWN, ElevatorPosition.SAFE_POSITION, GrabberState.OFF),
    STOWED(ArmPosition.UP, ElevatorPosition.BOTTOM, GrabberState.OFF);
    private final ArmPosition armPosition;
    private final ElevatorPosition elevatorPosition;
    private final GrabberState grabberState;

    CoralManipulatorState(
            ArmPosition armPosition, ElevatorPosition elevatorPosition, GrabberState grabberState) {
        this.armPosition = armPosition;
        this.elevatorPosition = elevatorPosition;
        this.grabberState = grabberState;
    }

    public ArmPosition getArmPosition() {
        return armPosition;
    }

    public ElevatorPosition getElevatorPosition() {
        return elevatorPosition;
    }

    public GrabberState getGrabberState() {
        return grabberState;
    }
}
