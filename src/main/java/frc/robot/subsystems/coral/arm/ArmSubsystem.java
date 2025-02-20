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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;
import frc.robot.util.state.StateUtils;
import frc.robot.util.state.StatefulSetpointSubsystem;

@Logged
public class ArmSubsystem extends StatefulSetpointSubsystem<ArmPosition, AngleUnit, Angle, MutAngle> implements SimulatableMechanism {
    private final TalonFX armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0)
            .withSlot(1);

    private final StatusSignal<Angle> armPosition = armKraken.getPosition();

    public ArmSubsystem() {
        super(ArmPosition.HOLD, StateUtils.mutableRotationSetpoint(), Units.Rotations.of(ArmConfig.ANGLE_TOLERANCE));
        armKraken.getConfigurator().apply(ArmConfig.talonFXConfiguration);
        CANcoder armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.CANIVORE_BUS);

        armEncoder.getConfigurator().apply(ArmConfig.cancoderConfiguration);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(armKraken, 0.001, armEncoder);
        }
    }

    @Override
    public Angle determineSetpoint(ArmPosition targetState) {
        return targetState == ArmPosition.HOLD ? armPosition.getValue() : targetState.getAngle();
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return armKraken.setControl(motionRequest.withPosition(setpoint).withSlot(1));
    }

    @Override
    public StatusSignal<Angle> currentStateSignal() {
        return armPosition;
    }

    @Override
    public double updateMechPos() {
        SmartDashboard.putNumber("Arm position", armKraken.getPosition().getValueAsDouble());
        return (armKraken.getPosition().getValueAsDouble() * 360.0) - 90;
    }
}
