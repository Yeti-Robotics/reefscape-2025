package frc.robot.subsystems.climber;

import static edu.wpi.first.wpilibj2.command.Commands.startEnd;
import static frc.robot.constants.Constants.*;
import static frc.robot.subsystems.climber.ClimberConfigs.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class Climber {
    private final TalonFX climberMotor;
    public CANcoder climberEncoder = new CANcoder(12);

    public Climber() {
        climberMotor = new TalonFX(climberId, RIO_BUS);
        var configurator = climberMotor.getConfigurator();
        configurator.apply(climberTalonFXConfigs);
    }

    private void stop() {
        climberMotor.stopMotor();
    }

    private void setSpeed(double speed) {
        climberMotor.set(speed);
    }

    public Command spinClimberForward(double speed) {
        return startEnd(() -> setSpeed(speed), this::stop);
    }

    public Command spinClimberReverse(double speed) {
        return startEnd(() -> setSpeed(-speed), this::stop);
    }
}
