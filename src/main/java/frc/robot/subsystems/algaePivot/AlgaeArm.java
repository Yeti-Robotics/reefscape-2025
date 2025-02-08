package frc.robot.subsystems.algaePivot;

import static frc.robot.subsystems.algaePivot.AlgaeConfigs.cancoderConfiguration;
import static frc.robot.subsystems.algaePivot.AlgaeConfigs.talonFXConfiguration;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;

public class AlgaeArm {
    private final TalonFX algaeArmKraken;
    final MotionMagicVoltage magicRequest;

    public enum Position {
        LOW(0), // placeholder
        MID(0), // placeholder
        HIGH(0); // placeholder

        private final int value;

        Position(final int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public AlgaeArm() {
        algaeArmKraken = new TalonFX(AlgaeConfigs.ROLLER_ID, Constants.CANIVORE_BUS);
        CANcoder armEncoder = new CANcoder(AlgaeConfigs.ROLLER_CANCODER_ID, Constants.CANIVORE_BUS);

        var armConfigurator = algaeArmKraken.getConfigurator();

        magicRequest = new MotionMagicVoltage(0);

        armConfigurator.apply(talonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();

        armEncoderConfigurator.apply(cancoderConfiguration);
    }

    public void moveUp(double speed) {
        algaeArmKraken.set(Math.abs(speed));
    }

    private void moveDown(double speed) {
        algaeArmKraken.set(-Math.abs(speed));
    }

    public void stop() {
        algaeArmKraken.stopMotor();
    }
}
