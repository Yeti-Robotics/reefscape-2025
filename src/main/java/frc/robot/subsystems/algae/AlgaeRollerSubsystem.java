package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.Constants.RIO_BUS;

public class AlgaeRollerSubsystem extends SubsystemBase {
    private final TalonFX roller;

    public AlgaeRollerSubsystem() {
        roller = new TalonFX(AlgaeRollerConfig.ROLLER_ID, RIO_BUS);

        var rollerConfigurator = roller.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = AlgaeRollerConfig.ROLLER_INVERSION;
        configs.MotorOutput.NeutralMode = AlgaeRollerConfig.ROLLER_NEUTRAL_MODE;
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
