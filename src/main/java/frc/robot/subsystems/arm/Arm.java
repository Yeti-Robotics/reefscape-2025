package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConfig.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;

@Logged
public class Arm extends SubsystemBase implements SimulatableMechanism {
    private final TalonFX armKraken;
    private final CANcoder armEncoder;
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
        armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.RIO_BUS);
        armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.RIO_BUS);

        var armConfigurator = armKraken.getConfigurator();

        magicRequest = new MotionMagicVoltage(0);

        armConfigurator.apply(talonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();

        armEncoderConfigurator.apply(cancoderConfiguration);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(armKraken, 0.001, armEncoder);
        }
    }

    public void moveUp(double speed) {
        armKraken.setControl(new DutyCycleOut(speed));
    }

    private void moveDown(double speed) {
        armKraken.setControl(new DutyCycleOut(speed));
    }

    public void stop() {
        armKraken.stopMotor();
    }

    public void target(Position position) {
        armKraken.setControl(
                magicRequest.withPosition(position.getValue()).withSlot(Robot.isReal() ? 0 : 1));
    }

    public Command targetCommand(double position) {
        return runOnce(
                () ->
                        armKraken.setControl(
                                magicRequest
                                        .withPosition(position)
                                        .withSlot(Robot.isReal() ? 0 : 1)));
    }

    public Command raiseArm() {
        return runEnd(() -> armKraken.set(1), this::stop);
    }

    public Command lowerArm() {
        return runEnd(() -> armKraken.set(-1), this::stop);
    }

    @Override
    public double updateMechPos() {
        return (armKraken.getPosition().getValueAsDouble() * 360.0) - 90;
    }
}
