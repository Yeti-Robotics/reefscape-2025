package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.StateManager;

public class Arm extends SubsystemBase {
    private final TalonFX armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
    private final MotionMagicTorqueCurrentFOC motionRequest = new MotionMagicTorqueCurrentFOC(0);

    private final StateManager<ArmPositions> armState = new StateManager<>(ArmPositions.STOWED, this);
    private final StatusSignal<Angle> armPosition = armKraken.getPosition();
    private double transitionTarget = 0;

    public Arm() {
        armKraken.getConfigurator().apply(ArmConfig.armMotorConfig);
        CANcoder armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.CANIVORE_BUS);

        armEncoder.getConfigurator().apply(ArmConfig.armCanCoderConfig);
    }

    @Override
    public void periodic() {
        armPosition.refresh();

        if (armState.isTransitioning()) {
            ArmPositions targetArmState = armState.transitioningTo().orElse(ArmPositions.HOLD);

            if (!armState.transitionStarted()) {
                armState.startTransition();

                double angle = switch (targetArmState) {
                    case L1:
                    case L2:
                    case L3:
                    case L4:
                        yield targetArmState.getAngle();
                    case HOLD:
                        yield armPosition.getValueAsDouble();
                    case STOWED:
                        yield 0;
                };

                StatusCode code = armKraken.setControl(motionRequest.withPosition(angle));

                if (!code.isOK()) {
                    armState.failTransition();
                } else {
                    transitionTarget = angle;
                }
            }

            if (atSetPoint(transitionTarget)) {
                armState.finishTransition();
            }
        }
    }

    private boolean atSetPoint(double angle) {
        return Math.abs(armPosition.getValueAsDouble() - angle) < ArmConfig.ANGLE_TOLERANCE;
    }
}
