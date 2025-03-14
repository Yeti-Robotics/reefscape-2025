package frc.robot.subsystems.coral.wrist;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.state.StateUtils;
import frc.robot.util.state.StatefulSetpointSubsystem;

@Logged
public class WristSubsystem
        extends StatefulSetpointSubsystem<WristPositions, AngleUnit, Angle, MutAngle> {
    private final TalonFX wristMotor = new TalonFX(WristConfigs.WRIST_KRAKEN_ID, Constants.RIO_BUS);
    private final CANcoder wristEncoder = new CANcoder(WristConfigs.WRIST_CANCODER_ID);
    private final StatusSignal<Angle> wristPosition = wristMotor.getPosition();
    private final PositionVoltage motionRequest = new PositionVoltage(0).withSlot(0);

    public WristSubsystem() {
        super(
                WristPositions.SAFE,
                StateUtils.mutableRotationSetpoint(),
                Units.Rotations.of(WristConfigs.WRIST_TOLERANCE));
        wristMotor.getConfigurator().apply(WristConfigs.wristMotorConfigs);
        wristEncoder.getConfigurator().apply(WristConfigs.wristEncoderConfigs);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(wristMotor, wristEncoder);
        }
        wristMotor.setPosition(WristPositions.SAFE.getAngle());
    }

    public Command moveWristHorizontal() {
        return runOnce(() -> moveTo(WristPositions.SAFE.getAngle()));
    }

    public Command moveWristVertical() {
        return runOnce(() -> moveTo(WristPositions.UNSAFE.getAngle()));
    }

    @Override
    public StatusSignal<Angle> currentStateSignal() {
        return wristPosition;
    }

    @Override
    public Angle determineSetpoint(WristPositions targetState) {
        return targetState == WristPositions.HOLD
                ? wristPosition.getValue()
                : targetState.getAngle();
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return wristMotor.setControl(motionRequest.withPosition(setpoint));
    }
}
