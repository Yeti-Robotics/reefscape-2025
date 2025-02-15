package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.coral.CoralIntake;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class Superstructure extends SubsystemBase {
    private CoralIntake coralIntake;
    private Arm arm;
    private ElevatorSubsystem elevator;

    public enum SuperState {
        IDLE,
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4,
        PREPARE_L1,
        PREPARE_L2,
        PREPARE_L3,
        PREPARE_L4,
        INTAKE_CORAL,
        INTAKE_ALGAE,
        PROCESS_ALGAE
    }

    private SuperState superState = SuperState.IDLE;

    public Superstructure(CoralIntake coralIntake, Arm arm, ElevatorSubsystem elevator) {
        this.coralIntake = coralIntake;
        this.arm = arm;
        this.elevator = elevator;

        new Trigger(this::occupiedIntake).whileTrue(coralIntake.spinClawForward());
    }

    public boolean occupiedIntake() {
        return coralIntake.isCoralInIntake();
    }
}
