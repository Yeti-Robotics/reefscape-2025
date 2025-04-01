package frc.robot.subsystems.coral.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.led.LEDConstants;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;
import frc.robot.util.state.StateUtils;
import frc.robot.util.state.StatefulSetpointSubsystem;

@Logged
public class ArmSubsystem extends StatefulSetpointSubsystem<ArmPosition, AngleUnit, Angle, MutAngle>
        implements SimulatableMechanism {
    private final TalonFX armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.RIO_BUS);
    private final NeutralOut neutralOut = new NeutralOut();
    private final MotionMagicTorqueCurrentFOC magicRequest =
            new MotionMagicTorqueCurrentFOC(0).withSlot(0);

    private final StatusSignal<Angle> armPosition = armKraken.getPosition();
    private final StatusSignal<Double> armTargetPos = armKraken.getClosedLoopReference();
    private CANcoder armEncoder;

    public ArmSubsystem() {
        super(
                ArmPosition.HOLD,
                StateUtils.mutableRotationSetpoint(),
                Units.Rotations.of(ArmConfig.ANGLE_TOLERANCE));
        armKraken.getConfigurator().apply(ArmConfig.talonFXConfiguration);
        armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.RIO_BUS);

        armEncoder.getConfigurator().apply(ArmConfig.cancoderConfiguration);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(armKraken, armEncoder);
        }
    }

    @Override
    public Angle determineSetpoint(ArmPosition targetState) {
        return targetState == ArmPosition.HOLD ? armPosition.getValue() : targetState.getAngle();
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return armKraken.setControl(magicRequest.withPosition(setpoint));
    }

    @Override
    public StatusSignal<Angle> currentStateSignal() {
        return armPosition;
    }

    @Override
    public Angle getCurrentPosition() {
        return armPosition.getValue();
    }

    @Override
    public Angle getTargetPosition() {
        return Units.Rotations.of(armTargetPos.getValue());
    }

    @Override
    public boolean isTransitionFinished() {
        boolean transtionFinished = super.isTransitionFinished();
        if (transtionFinished
                && transitioningTo().isPresent()
                && transitioningTo().get().equals(ArmPosition.GROUND)) {
            armKraken.setControl(neutralOut);
        }
        return transtionFinished;
    }

    public boolean isEncoderZeroed() {
        double position = armEncoder.getPosition().refresh().getValueAsDouble();
        return position >= 0 && position <= LEDConstants.ZERO_TOLERANCE;
    }
}
