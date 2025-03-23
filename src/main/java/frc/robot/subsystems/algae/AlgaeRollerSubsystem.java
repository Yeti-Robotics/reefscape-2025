package frc.robot.subsystems.algae;

import static frc.robot.constants.Constants.*;
import static frc.robot.constants.Constants.RIO_BUS;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRollerSubsystem extends SubsystemBase {
    private final TalonFX roller = new TalonFX(AlgaeRollerConfig.ROLLER_ID, RIO_BUS);

    public AlgaeRollerSubsystem() {
        roller.getConfigurator().apply(AlgaeRollerConfig.TALON_FX_CONFIGURATION);
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
