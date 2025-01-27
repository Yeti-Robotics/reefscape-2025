package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.CoralIntake;

public class Superstructure extends SubsystemBase {
    private CoralIntake coralIntake;

    public enum SuperState{
        IDLE,
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4,
        INTAKE_CORAL,
        INTAKE_ALGAE,
        PROCESS_ALGAE,
        NET_ALGAE,
        AIM_REEF,
        AIM_HP
    }

    private SuperState superState = SuperState.IDLE;

    public Superstructure(
            CoralIntake coralIntake
    ) {
        this.coralIntake = coralIntake;
    }

    @Override
    public void periodic(){

    }


}
