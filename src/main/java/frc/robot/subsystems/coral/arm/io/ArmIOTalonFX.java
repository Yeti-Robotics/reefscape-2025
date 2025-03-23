package frc.robot.subsystems.coral.arm.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.util.akit.LoggingUtils;
import frc.robot.util.akit.device.inputs.CANCoderDeviceInputs;
import frc.robot.util.akit.device.inputs.CANCoderDeviceInputsAutoLogged;
import frc.robot.util.akit.device.inputs.TalonFXDeviceInputs;
import frc.robot.util.akit.device.inputs.TalonFXDeviceInputsAutoLogged;
import frc.robot.util.akit.device.log.DeviceLogger;
import frc.robot.util.device.impl.CANCoderDevice;
import frc.robot.util.device.impl.TalonFXMotor;

public class ArmIOTalonFX implements ArmIO {
    private final MotionMagicTorqueCurrentFOC motionMagicReq = new MotionMagicTorqueCurrentFOC(0);

    final CANcoder armCancoder =
            CANCoderDevice.configure(ArmConfig.ARM_CANCODER_ID, Constants.CANIVORE_BUS)
                    .syncConfigs()
                    .getDevice();

    final TalonFX armMotor =
            TalonFXMotor.configure(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS)
                    .usingFusedCANcoder(armCancoder)
                    .syncConfigs()
                    .getDevice();

    private final DeviceLogger<TalonFXDeviceInputs> talonLogger = DeviceLogger.forDevice(armMotor);
    private final DeviceLogger<CANCoderDeviceInputs> canCoderLogger =
            DeviceLogger.forDevice(armCancoder);

    private final TalonFXDeviceInputsAutoLogged talonInputs = new TalonFXDeviceInputsAutoLogged();
    private final CANCoderDeviceInputsAutoLogged cancoderInputs = new CANCoderDeviceInputsAutoLogged();
    private final StatusSignal<Angle> positionSignal = armMotor.getPosition();

    private ArmPosition currentSetpoint = null;

    @Override
    public void updateInputs(ArmInputs inputs) {
        LoggingUtils.logInputs("ArmSubsystem/ArmMotor", talonLogger, talonInputs);
        LoggingUtils.logInputs("ArmSubsystem/ArmCancoder", canCoderLogger, cancoderInputs);
    }

    @Override
    public Angle getPosition() {
        return positionSignal.getValue();
    }

    @Override
    public void setPosition(Angle value) {
        armMotor.setControl(motionMagicReq.withPosition(value));
    }

    @Override
    public void toSetpoint(ArmPosition setpoint) {
        currentSetpoint = setpoint;
        ArmIO.super.toSetpoint(setpoint);
    }
}
