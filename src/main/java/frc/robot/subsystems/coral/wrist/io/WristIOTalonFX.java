package frc.robot.subsystems.coral.wrist.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.util.akit.device.can.cancoder.CANCoderDevice;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

public class WristIOTalonFX implements WristIO {
    private final CANcoder wristCancoder = CANCoderDevice.configure(WristConfig.WRIST_CANCODER_ID, Constants.CANIVORE_BUS)
            .log("Wrist/WristCancoder")
            .withConfig(WristConfig.wristEncoderConfigs)
            .syncConfigs()
            .getDevice();

    private final TalonFX wristMotor = TalonFXDevice.configure(WristConfig.WRIST_KRAKEN_ID, Constants.CANIVORE_BUS)
            .log("Wrist/WristMotor")
            .withConfig(WristConfig.wristMotorConfigs)
            .withFusedCANcoder(wristCancoder)
            .withStatusSignalFrequency(HardwareConstants.SETPOINT_UPDATE_FREQUENCY, TalonFX::getPosition)
            .syncConfigs()
            .getDevice();

    private final StatusSignal<Angle> positionSignal = wristMotor.getPosition();
    private final PositionVoltage motorReq = new PositionVoltage(0);

    @Override
    public Angle getState() {
        return positionSignal.getValue();
    }

    @Override
    public void setState(Angle value) {
        wristMotor.setControl(motorReq.withPosition(value));
    }
}
