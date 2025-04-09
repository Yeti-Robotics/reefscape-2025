package frc.robot.subsystems.climber.io;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX climberMotor =
            TalonFXDevice.configure(ClimberConfig.CLIMBER_MOTOR_ID, Constants.RIO_BUS)
                    .using(ClimberConfig.climberTalonFXConfigs)
                    .log("ClimberIO/Motor")
                    .getDevice();

    @Override
    public void spinSpeed(double speed) {
        climberMotor.set(speed);
    }

    @Override
    public void stop() {
        climberMotor.stopMotor();
    }
}
