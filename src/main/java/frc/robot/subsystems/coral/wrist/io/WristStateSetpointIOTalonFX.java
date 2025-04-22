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

public class WristStateSetpointIOTalonFX implements WristStateSetpointIO {
    final CANcoder wristCancoder =
            CANCoderDevice.configure(WristConfigs.WRIST_CANCODER_ID, Constants.CANIVORE_BUS)
                    .log("WristIO/WristCancoder")
                    .using(WristConfigs.wristEncoderConfigs)
                    .syncConfigs()
                    .getDevice();

    final TalonFX wristMotor =
            TalonFXDevice.configure(WristConfigs.WRIST_KRAKEN_ID, Constants.CANIVORE_BUS)
                    .log("WristIO/WristMotor")
                    .using(WristConfigs.wristMotorConfigs)
                    .withFusedCANcoder(wristCancoder)
                    .withStatusSignalFrequency(
                            HardwareConstants.SETPOINT_UPDATE_FREQUENCY, TalonFX::getPosition)
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
