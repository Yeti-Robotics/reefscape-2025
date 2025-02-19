package frc.robot.subsystems.algae;

import static frc.robot.subsystems.algae.AlgaeArmConfigs.cancoderConfiguration;
import static frc.robot.subsystems.algae.AlgaeArmConfigs.talonFXConfiguration;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class AlgaeArm extends SubsystemBase {
    private final TalonFX algaeArmKraken;
    final MotionMagicVoltage magicRequest;
    public CANcoder armEncoder;
    boolean armZero = false;

    public AlgaeArm() {
        algaeArmKraken = new TalonFX(AlgaeArmConfigs.ALGAE_ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
        armEncoder = new CANcoder(AlgaeArmConfigs.ALGAE_ARM_CANCODER_ID, Constants.CANIVORE_BUS);

        var armConfigurator = algaeArmKraken.getConfigurator();

        magicRequest = new MotionMagicVoltage(0);

        armConfigurator.apply(talonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();

        armEncoderConfigurator.apply(cancoderConfiguration);

        SmartDashboard.putData(new InstantCommand(() -> armZero = !armZero));
    }

    public void stop() {
        algaeArmKraken.stopMotor();
    }

    public void target(AlgaePosition position) {
        algaeArmKraken.setControl(magicRequest.withPosition(position.getValue()));
    }

    public boolean isArmZero() {
        return Utils.isSimulation() ? armZero : isEncoderZeroed(armEncoder);
    }

    public boolean isEncoderZeroed(CANcoder encoder) {
        double position = encoder.getPosition().refresh().getValueAsDouble();
        return position >= 0 || position <= Constants.ZERO_TOLERANCE;
    }

    private void setAlgaeArmKrakenSpeed(double speed) {
        algaeArmKraken.set(speed);
    }

    public Command spinRoller(double speed) {
        return startEnd(() -> setAlgaeArmKrakenSpeed(speed), this::stop);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isArmZero", isArmZero());
    }
}
