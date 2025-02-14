package frc.robot.subsystems.algaePivot;

import static frc.robot.constants.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private final TalonFX roller;

    public AlgaeIntake() {
        roller = new TalonFX(AlgaeConfigs.ROLLER_ID, RIO_BUS);

        var rollerConfigurator = roller.getConfigurator();
        rollerConfigurator.apply(AlgaeConfigs.configs);
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
