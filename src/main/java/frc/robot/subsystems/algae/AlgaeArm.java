package frc.robot.subsystems.algae;

import static frc.robot.subsystems.algae.AlgaeArmConfigs.cancoderConfiguration;
import static frc.robot.subsystems.algae.AlgaeArmConfigs.talonFXConfiguration;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;

public class AlgaeArm extends SubsystemBase {
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
        algaeArmKraken = new TalonFX(AlgaeArmConfigs.ALGAE_ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
        CANcoder armEncoder =
                new CANcoder(AlgaeArmConfigs.ALGAE_ARM_CANCODER_ID, Constants.CANIVORE_BUS);

        var armConfigurator = algaeArmKraken.getConfigurator();

        magicRequest = new MotionMagicVoltage(0);

        armConfigurator.apply(talonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();

        armEncoderConfigurator.apply(cancoderConfiguration);
    }

    public void stop() {
        algaeArmKraken.stopMotor();
    }

    public void target(Arm.Position position) {
        algaeArmKraken.setControl(magicRequest.withPosition(position.getValue()));
    }

    private void setAlgaeArmKrakenSpeed(double speed) {
        algaeArmKraken.set(speed);
    }

    public Command spinRoller(double speed) {
        return startEnd(() -> setAlgaeArmKrakenSpeed(speed), this::stop);
    }
}
