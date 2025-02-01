package frc.robot.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ElevatorConfigs {
    static int ELEVATOR_ID = 0;
    static final double ELEVATOR_POSITION_STATUS_FRAME = 0;
    static final double ELEVATOR_VELOCITY_STATUS_FRAME = 0;

    static final double ELEVATOR_P = 0;
    static final double ELEVATOR_I = 0;
    static final double ELEVATOR_D = 0;

    static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(ELEVATOR_P).withKI(ELEVATOR_I).withKD(ELEVATOR_D);

    static final double MAGNET_OFFSET= 0;

    static final double GEAR_RATIO = 0;
}

