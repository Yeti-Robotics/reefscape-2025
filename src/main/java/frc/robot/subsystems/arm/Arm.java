package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConfig.cancoderConfiguration;
import static frc.robot.subsystems.arm.ArmConfig.talonFXConfiguration;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;

@Logged
public class Arm extends SubsystemBase {
    private final TalonFX armKraken;
    final MotionMagicVoltage magicRequest;

    public enum Position {
        LOW(30), // placeholder
        MID(60), // placeholder
        HIGH(90); // placeholder

        private final int value;

        Position(final int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public Arm() {
        armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
        CANcoder armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.CANIVORE_BUS);

        var armConfigurator = armKraken.getConfigurator();

        magicRequest = new MotionMagicVoltage(0);

        armConfigurator.apply(talonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();

        armEncoderConfigurator.apply(cancoderConfiguration);
    }

    public void moveUp(double speed) {
        armKraken.set(Math.abs(speed));
    }

    private void moveDown(double speed) {
        armKraken.set(-Math.abs(speed));
    }

    public void stop() {
        armKraken.stopMotor();
    }

    public void target(Position position) {
        armKraken.setControl(magicRequest.withPosition(position.getValue()));
    }
}
