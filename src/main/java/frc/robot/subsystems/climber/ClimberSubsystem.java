package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import static frc.robot.subsystems.climber.ClimberConfig.*;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climber;
    private final CANcoder cancoder;
    final MotionMagicVoltage magicRequest;

    public ClimberSubsystem() {
        climber = new TalonFX(climberId, Constants.RIO_BUS);
        cancoder = new CANcoder(canCoderId, Constants.RIO_BUS);
        var climberConfigurator = climber.getConfigurator();
        climberConfigurator.apply(climberTalonFXConfigs);
        var cancoderConfigurator = cancoder.getConfigurator();
        cancoderConfigurator.apply(cancoderConfiguration);
        magicRequest = new MotionMagicVoltage(0);
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

    public void target(ClimberPosition position) {
        climber.setControl(magicRequest.withPosition(position.getAngle()));
    }
}
