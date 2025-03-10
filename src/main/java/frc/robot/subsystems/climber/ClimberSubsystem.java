package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConfig.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

@Logged
public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climber;
    private final CANcoder cancoder;
    final DutyCycleOut magicRequest;

    public ClimberSubsystem() {
        climber = new TalonFX(climberId, Constants.RIO_BUS);
        cancoder = new CANcoder(canCoderId, Constants.RIO_BUS);
        magicRequest = new DutyCycleOut(0);

        climber.getConfigurator().apply(climberTalonFXConfigs);
        cancoder.getConfigurator().apply(cancoderConfiguration);
    }

    private void setClimberSpeed(double speed) {
        climber.set(speed);
    }

    private void stop() {
        climber.stopMotor();
    }

    public Command spinClimber(double speed) {
        return startEnd(() -> setClimberSpeed(speed), this::stop);
    }

    public void move(double speed) {
        climber.setControl(new DutyCycleOut(speed));
    }
}
