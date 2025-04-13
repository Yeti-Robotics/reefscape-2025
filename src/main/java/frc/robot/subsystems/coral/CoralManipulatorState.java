package frc.robot.subsystems.coral;

import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.subsystems.coral.wrist.WristPositions;

public enum CoralManipulatorState {
    GROUND_INTAKE(
            ArmPosition.GROUND,
            ElevatorPosition.BOTTOM,
            GrabberState.ROLL_IN,
            WristPositions.UNSAFE),
    ALGAE_GROUND(
            ArmPosition.AWAY,
            ElevatorPosition.BOTTOM,
            GrabberState.ALGAE_ROLL_IN,
            WristPositions.UNSAFE),
    BARGE(
            ArmPosition.ALGAE_SCORE,
            ElevatorPosition.POS_L4,
            GrabberState.ALGAE_HOLD,
            WristPositions.UNSAFE),
    SHOOT_BARGE(
            ArmPosition.ALGAE_SCORE,
            ElevatorPosition.POS_L4,
            GrabberState.ROLL_OUT,
            WristPositions.UNSAFE),
    L1(ArmPosition.POS_L1, ElevatorPosition.POS_L1, GrabberState.OFF, WristPositions.UNSAFE),
    SCORE_L1(
            ArmPosition.POS_L1,
            ElevatorPosition.POS_L1,
            GrabberState.ROLL_OUT,
            WristPositions.UNSAFE),
    L2(ArmPosition.POS_L2, ElevatorPosition.POS_L2, GrabberState.OFF, WristPositions.SAFE),
    SCORE_L2(
            ArmPosition.SCORE_L2,
            ElevatorPosition.POS_L2,
            GrabberState.ROLL_OUT,
            WristPositions.SAFE),
    L3(ArmPosition.POS_L3, ElevatorPosition.POS_L3, GrabberState.OFF, WristPositions.SAFE),
    SCORE_L3(
            ArmPosition.SCORE_L3, ElevatorPosition.SCORE_L3, GrabberState.OFF, WristPositions.SAFE),
    L4(ArmPosition.POS_L4, ElevatorPosition.POS_L4, GrabberState.OFF, WristPositions.SAFE),
    CLIMB_L4(ArmPosition.CLIMB_L4, ElevatorPosition.POS_L4, GrabberState.OFF, WristPositions.SAFE),
    SCORE_L4(
            ArmPosition.SCORE_L4, ElevatorPosition.SCORE_L4, GrabberState.OFF, WristPositions.SAFE),
    SCORE_CLIMB_L4(
            ArmPosition.SCORE_CLIMB_L4,
            ElevatorPosition.POS_L4,
            GrabberState.OFF,
            WristPositions.SAFE),
    HP_INTAKE(ArmPosition.HP, ElevatorPosition.BOTTOM, GrabberState.ROLL_IN, WristPositions.UNSAFE),
    ALGAE_LOW(
            ArmPosition.AWAY,
            ElevatorPosition.LOW_ALGAE,
            GrabberState.ROLL_IN,
            WristPositions.UNSAFE),
    STOWED(ArmPosition.UP, ElevatorPosition.BOTTOM, GrabberState.OFF, WristPositions.SAFE),
    CLIMB(ArmPosition.GROUND, ElevatorPosition.BOTTOM, GrabberState.OFF, WristPositions.UNSAFE),
    LOLLIPOP(
            ArmPosition.GROUND, ElevatorPosition.BOTTOM, GrabberState.ROLL_IN, WristPositions.SAFE),
    ALGAE_HIGH(
            ArmPosition.HIGH_ALGAE,
            ElevatorPosition.HIGH_ALGAE,
            GrabberState.ROLL_IN,
            WristPositions.UNSAFE),
    IDLE(ArmPosition.HOLD, ElevatorPosition.HOLD, GrabberState.OFF, WristPositions.SAFE),
    DISABLED(ArmPosition.HOLD, ElevatorPosition.HOLD, GrabberState.OFF, WristPositions.SAFE),
    ALGAE_STOW(
            ArmPosition.ALGAE_STOW,
            ElevatorPosition.BOTTOM,
            GrabberState.ALGAE_HOLD,
            WristPositions.UNSAFE);

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
