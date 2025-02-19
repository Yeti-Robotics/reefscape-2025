package frc.robot.subsystems.climber;

import static edu.wpi.first.wpilibj2.command.Commands.startEnd;
import static frc.robot.constants.Constants.*;
import static frc.robot.subsystems.climber.ClimberConfigs.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {
    private final TalonFX climberMotor;
    public CANcoder climberEncoder = new CANcoder(12);
    boolean climberZero = false;

    public Climber() {
        climberMotor = new TalonFX(climberId, RIO_BUS);
        var configurator = climberMotor.getConfigurator();
        configurator.apply(climberTalonFXConfigs);
        SmartDashboard.putData(new InstantCommand(() -> climberZero = !climberZero));
    }

    public boolean isClimberZero() {
        return Utils.isSimulation() ? climberZero : isEncoderZeroed(climberEncoder);
    }

    public boolean isEncoderZeroed(CANcoder encoder) {
        double position = encoder.getPosition().refresh().getValueAsDouble();
        return position >= 0 || position <= Constants.ZERO_TOLERANCE;
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

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isClimberZero", isClimberZero());
    }
}
