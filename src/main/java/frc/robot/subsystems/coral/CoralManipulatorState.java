package frc.robot.subsystems.coral;

import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.subsystems.coral.wrist.WristPosition;

public enum CoralManipulatorState {
    GROUND_INTAKE(
            ArmPosition.GROUND,
            ElevatorPosition.BOTTOM,
            GrabberState.ROLL_IN,
            WristPosition.UNSAFE),
    L1(ArmPosition.POS_L1, ElevatorPosition.POS_L1, GrabberState.OFF, WristPosition.UNSAFE),
    SCORE_L1(
            ArmPosition.POS_L1,
            ElevatorPosition.POS_L1,
            GrabberState.ROLL_OUT,
            WristPosition.UNSAFE),
    L2(ArmPosition.POS_L2, ElevatorPosition.POS_L2, GrabberState.OFF, WristPosition.SAFE),
    SCORE_L2(ArmPosition.SCORE_L2, ElevatorPosition.POS_L2, GrabberState.OFF, WristPosition.SAFE),
    L3(ArmPosition.POS_L3, ElevatorPosition.POS_L3, GrabberState.OFF, WristPosition.SAFE),
    SCORE_L3(ArmPosition.SCORE_L3, ElevatorPosition.SCORE_L3, GrabberState.OFF, WristPosition.SAFE),
    L4(ArmPosition.POS_L4, ElevatorPosition.POS_L4, GrabberState.OFF, WristPosition.SAFE),
    SCORE_L4(ArmPosition.SCORE_L4, ElevatorPosition.SCORE_L4, GrabberState.OFF, WristPosition.SAFE),
    HP_INTAKE(ArmPosition.HP, ElevatorPosition.BOTTOM, GrabberState.ROLL_IN, WristPosition.UNSAFE),
    STOWED(ArmPosition.UP, ElevatorPosition.BOTTOM, GrabberState.OFF, WristPosition.SAFE),
    CLIMB(ArmPosition.GROUND, ElevatorPosition.BOTTOM, GrabberState.OFF, WristPosition.SAFE),
    ALGAE_HIGH(
            ArmPosition.POS_L3,
            ElevatorPosition.POS_L3,
            GrabberState.ROLL_IN,
            WristPosition.UNSAFE);

    private final ArmPosition armPosition;
    private final ElevatorPosition elevatorPosition;
    private final GrabberState grabberState;
    private final WristPosition wristPosition;

    CoralManipulatorState(
            ArmPosition armPosition,
            ElevatorPosition elevatorPosition,
            GrabberState grabberState,
            WristPosition wristPosition) {
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

    public WristPosition getWristPosition() {
        return wristPosition;
    }
}
