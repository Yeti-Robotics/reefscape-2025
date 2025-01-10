package frc.robot;

import static frc.robot.constants.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private final TalonFX roller;

    public static class IntakeConstants {
        public static final int ROLLER_ID = 8;
        public static final InvertedValue ROLLER_INVERSION = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue ROLLER_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final double POSITION_STATUS_FRAME = 0.05;
        public static final double VELOCITY_STATUS_FRAME = 0.01;
    }

    public AlgaeIntake() {
        roller = new TalonFX(IntakeConstants.ROLLER_ID, RIO_BUS);

        var rollerConfigurator = roller.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = IntakeConstants.ROLLER_INVERSION;
        configs.MotorOutput.NeutralMode = IntakeConstants.ROLLER_NEUTRAL_MODE;
        rollerConfigurator.apply(configs);
    }

    private void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    private void stop() {
        roller.stopMotor();
    }

    public Command spinRoller(double speed) {
        return startEnd(() -> setRollerSpeed(speed), this::stop);
    }
}
