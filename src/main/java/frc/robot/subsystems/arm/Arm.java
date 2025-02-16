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
import frc.robot.util.sim.Simulatable;

@Logged
public class Arm extends SubsystemBase implements Simulatable {
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
        armKraken.setControl(magicRequest.withPosition(position.getValue()));
    }

    public Command raiseArm() {
        return runEnd(() -> armKraken.set(1), this::stop);
    }

    public Command lowerArm() {
        return runEnd(() -> armKraken.set(-1), this::stop);
    }

    @Override
    public double update() {
        return armKraken.getPosition().getValueAsDouble();
    }
}
