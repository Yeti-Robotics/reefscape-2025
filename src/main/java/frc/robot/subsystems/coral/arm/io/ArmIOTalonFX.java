package frc.robot.subsystems.coral.arm.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.util.akit.device.can.cancoder.CANCoderDevice;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;
import jakarta.inject.Singleton;

@Singleton
public class ArmIOTalonFX implements ArmIO {
    private final MotionMagicTorqueCurrentFOC motionMagicReq = new MotionMagicTorqueCurrentFOC(0);

    final CANcoder armCancoder =
            CANCoderDevice.configure(ArmConfig.ARM_CANCODER_ID, Constants.CANIVORE_BUS)
                    .log("ArmIO/Cancoder")
                    .withConfig(ArmConfig.cancoderConfiguration)
                    .syncConfigs()
                    .getDevice();

    final TalonFX armMotor =
            TalonFXDevice.configure(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS)
                    .log("ArmIO/Motor")
                    .withConfig(ArmConfig.talonFXConfiguration)
                    .withFusedCANcoder(armCancoder)
                    .withStatusSignalFrequency(
                            HardwareConstants.SETPOINT_UPDATE_FREQUENCY, TalonFX::getPosition)
                    .syncConfigs()
                    .getDevice();

    final StatusSignal<Angle> positionSignal = armMotor.getPosition();

    @Override
    public Angle getState() {
        return positionSignal.getValue();
    }

    @Override
    public void setState(Angle value) {
        armMotor.setControl(motionMagicReq.withPosition(value));
    }
}
