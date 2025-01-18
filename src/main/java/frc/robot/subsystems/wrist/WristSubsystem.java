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

    public enum Positions {
        HORIZONTAL(0),
        VERTICAL(90);

        private final double angle;

        Positions(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public WristSubsystem() {
        wristMotor = new TalonFX(WristConfigs.deviceId, Constants.RIO_BUS);
        wristEncoder = new CANcoder(WristConfigs.deviceId);

        var wristConfigurator = wristMotor.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = WristConfigs.motorInversion;
        configs.MotorOutput.NeutralMode = WristConfigs.neutralMode;

        configs.Slot0.kP = 2;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.MotionMagic.MotionMagicAcceleration = 1;
        configs.MotionMagic.MotionMagicCruiseVelocity = .04;

        wristConfigurator.apply(configs);
    }

    public void setPosition(Positions position) {
        double targetEncoderUnits = degreesToRotations(position.getAngle());
        wristMotor.setControl(new MotionMagicVoltage(targetEncoderUnits));
    }

    public void resetEncoder() {
        wristEncoder.setPosition(0);
    }

    // not sure abt this again, but found it on the internet again :sob:
    private double degreesToRotations(double degrees) {
        double gearRatio = 20.0 / 75.0;
        return (degrees / 360.0) * gearRatio;
    }

    public Command moveToPosition(Positions position) {
        return runOnce(() -> setPosition(position));
    }
}
