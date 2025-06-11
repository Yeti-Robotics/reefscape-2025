package frc.robot.subsystems.coral.wrist.io;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;
import frc.robot.util.akit.device.can.CANUtil;
import frc.robot.util.akit.device.can.cancoder.CANCoderDevice;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

import java.util.function.Supplier;

public class WristIOTalonFX implements WristIO {
    private final TalonFXDevice wristMotor = TalonFXDevice.configure(WristConfig.WRIST_KRAKEN_ID, Constants.CANIVORE_BUS)
            .log("Wrist/WristMotor")
            .withConfig(WristConfig.wristMotorConfigs)
            .withFusedCANcoder(CANCoderDevice.configure(
                            WristConfig.WRIST_CANCODER_ID, Constants.CANIVORE_BUS)
                    .log("Wrist/WristCANcoder")
                    .withConfig(WristConfig.wristEncoderConfigs)
                    .syncConfigs()
                    .getDevice())
            .syncConfigs();

    private final Supplier<Angle> wristPosition = wristMotor.positionSupplier(CANUtil.TALON_MAX_UPDATE_HZ);
    private final PositionVoltage motorReq = new PositionVoltage(0);

    @Override
    public Angle getState() {
        return wristPosition.get();
    }

    @Override
    public void setState(Angle value) {
        wristMotor.getDevice().setControl(motorReq.withPosition(value));
    }
}
