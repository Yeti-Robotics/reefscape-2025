package frc.robot.subsystems.coral;

import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.grabber.GrabberState;

public enum CoralManipulatorState {
    READY(ArmPosition.STOWED, ElevatorPosition.HOVER, GrabberState.OFF),
    INTAKE(ArmPosition.STOWED, ElevatorPosition.BOTTOM, GrabberState.ROLL_IN),
    L1(ArmPosition.L1, ElevatorPosition.L1, GrabberState.OFF),
    L2(ArmPosition.L2, ElevatorPosition.L2, GrabberState.OFF),
    L3(ArmPosition.L3, ElevatorPosition.L3, GrabberState.OFF),
    L4(ArmPosition.L4, ElevatorPosition.L4, GrabberState.OFF),
    IDLE(ArmPosition.HOLD, ElevatorPosition.HOLD, GrabberState.OFF),
    PLACE(ArmPosition.HOLD, ElevatorPosition.HOVER, GrabberState.ROLL_OUT);

    private final ArmPosition armPosition;
    private final ElevatorPosition elevatorPosition;
    private final GrabberState grabberState;

    CoralManipulatorState(ArmPosition armPosition, ElevatorPosition elevatorPosition, GrabberState grabberState) {
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
