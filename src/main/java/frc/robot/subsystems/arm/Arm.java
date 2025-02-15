package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConfig.cancoderConfiguration;
import static frc.robot.subsystems.arm.ArmConfig.talonFXConfiguration;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.StateManager;

public class Arm extends SubsystemBase {
    private final TalonFX armKraken;
    final MotionMagicVoltage magicRequest;

    public StateManager<ArmPositions> armState = new StateManager<>(ArmPositions.STOWED);


    public Arm() {
        armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
        CANcoder armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.CANIVORE_BUS);

        var armConfigurator = armKraken.getConfigurator();

        magicRequest = new MotionMagicVoltage(0);

        armConfigurator.apply(talonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();

        armEncoderConfigurator.apply(cancoderConfiguration);
    }

    @Override
    public void periodic() {
        ArmPositions positions = armState.getState();
        ArmPositions targetPosition = armState.getTargetState();
        // if i am transitioning to a new state, set the target position
        if (armState.isTransitioning()) {
            target(targetPosition);
        }
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

    public void target(ArmPositions position) {
        armKraken.setControl(magicRequest.withPosition(position.getAngle()));
    }

    public Command moveTo(ArmPositions targetPosition){
        return runOnce(() -> armState.transitionTo(targetPosition));
    }
}
