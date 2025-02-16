package frc.robot.subsystems.algae;

import static frc.robot.constants.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRoller extends SubsystemBase {
    private final TalonFX roller;

    public AlgaeRoller() {
        roller = new TalonFX(AlgaeRollerConfigs.ROLLER_ID, RIO_BUS);

        var rollerConfigurator = roller.getConfigurator();
        rollerConfigurator.apply(AlgaeRollerConfigs.configs);
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
