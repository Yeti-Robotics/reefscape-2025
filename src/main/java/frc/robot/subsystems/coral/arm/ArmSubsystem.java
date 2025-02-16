package frc.robot.subsystems.coral.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;
import frc.robot.util.state.StatefulSetpointSubsystem;

public class ArmSubsystem extends StatefulSetpointSubsystem<ArmPosition, AngleUnit, Angle> {
    private final TalonFX armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
    private final MotionMagicTorqueCurrentFOC motionRequest = new MotionMagicTorqueCurrentFOC(0);

    private final StatusSignal<Angle> armPosition = armKraken.getPosition();

    public ArmSubsystem() {
        super(ArmPosition.HOLD);
        armKraken.getConfigurator().apply(ArmConfig.armMotorConfig);
        CANcoder armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.CANIVORE_BUS);

        armEncoder.getConfigurator().apply(ArmConfig.armCanCoderConfig);
    }

    @Override
    public Angle determineSetpoint(ArmPosition targetState) {
        return Units.Degrees.of(targetState.getAngle());
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return armKraken.setControl(motionRequest.withPosition(setpoint));
    }

    @Override
    public StatusSignal<Angle> setPointSignal() {
        return armPosition;
    }

    @Override
    public Angle getErrorTolerance() {
        return Units.Degrees.of(ArmConfig.ANGLE_TOLERANCE);
    }
}
