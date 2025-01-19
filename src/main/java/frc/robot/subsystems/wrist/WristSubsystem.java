package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class WristSubsystem extends SubsystemBase {
    private final TalonFX wristMotor;
    private final CANcoder wristEncoder;

    public enum WristPositions {
        HORIZONTAL(0),
        VERTICAL(90);

        private final double angle;

        WristPositions(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public WristSubsystem() {
        wristMotor = new TalonFX(WristConfigs.DEVICE_ID, Constants.RIO_BUS);
        wristEncoder = new CANcoder(WristConfigs.DEVICE_ID);

        var wristConfigurator = wristMotor.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = WristConfigs.MOTOR_INVERSION;
        configs.MotorOutput.NeutralMode = WristConfigs.NEUTRAL_MODE_VALUE;

        configs.Slot0.kP = 2;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.MotionMagic.MotionMagicAcceleration = 1;
        configs.MotionMagic.MotionMagicCruiseVelocity = .04;
        configs.MotionMagic.MotionMagicJerk = 20;
        configs.Feedback.RotorToSensorRatio = 1;
        configs.Feedback.SensorToMechanismRatio = 3.75;

        wristConfigurator.apply(configs);
    }

    public void setPosition(WristPositions position) {
        wristMotor.setControl(new MotionMagicVoltage(position.getAngle()));
    }

    public void resetEncoder() {
        wristEncoder.setPosition(0);
    }

    public Command moveToPosition(WristPositions position) {
        return runOnce(() -> setPosition(position));
    }
}
