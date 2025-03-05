package frc.robot.subsystems.coral;

import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.subsystems.coral.wrist.WristPositions;

public enum CoralManipulatorState {
    INTAKE_CORAL(
            ArmPosition.DOWN,
            ElevatorPosition.INTAKE,
            GrabberState.ROLL_IN,
            WristPositions.VERTICAL),
    L1(ArmPosition.POS_L1, ElevatorPosition.POS_L1, GrabberState.ROLL_OUT, WristPositions.VERTICAL),
    L2(ArmPosition.POS_L2, ElevatorPosition.POS_L2, GrabberState.OFF, WristPositions.HORIZONTAL),
    SCORE_L2(
            ArmPosition.SCORE_L2,
            ElevatorPosition.POS_L2,
            GrabberState.ROLL_OUT,
            WristPositions.HORIZONTAL),
    L3(
            ArmPosition.POS_L3,
            ElevatorPosition.SAFE_POSITION,
            GrabberState.OFF,
            WristPositions.HORIZONTAL),
    SCORE_L3(
            ArmPosition.SCORE_L3,
            ElevatorPosition.POS_L3,
            GrabberState.ROLL_OUT,
            WristPositions.HORIZONTAL),
    L4(ArmPosition.POS_L4, ElevatorPosition.POS_L4, GrabberState.OFF, WristPositions.HORIZONTAL),
    SCORE_L4(
            ArmPosition.SCORE_L4,
            ElevatorPosition.POS_L4,
            GrabberState.ROLL_OUT,
            WristPositions.HORIZONTAL),
    SCOREREEF(
            ArmPosition.DOWN,
            ElevatorPosition.SAFE_POSITION,
            GrabberState.OFF,
            WristPositions.HORIZONTAL),
    STOWED(ArmPosition.UP, ElevatorPosition.BOTTOM, GrabberState.OFF, WristPositions.HORIZONTAL),
    IDLE(ArmPosition.HOLD, ElevatorPosition.HOLD, GrabberState.OFF, WristPositions.HORIZONTAL);

    private final ArmPosition armPosition;
    private final ElevatorPosition elevatorPosition;
    private final GrabberState grabberState;
    private final WristPositions wristPosition;

    CoralManipulatorState(
            ArmPosition armPosition,
            ElevatorPosition elevatorPosition,
            GrabberState grabberState,
            WristPositions wristPosition) {
        this.armPosition = armPosition;
        this.elevatorPosition = elevatorPosition;
        this.grabberState = grabberState;
        this.wristPosition = wristPosition;
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

    public WristPositions getWristPosition() {
        return wristPosition;
    }
}
