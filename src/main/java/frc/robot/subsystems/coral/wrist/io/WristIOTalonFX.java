package frc.robot.subsystems.coral.wrist.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;
import frc.robot.util.akit.device.impl.CANCoderDevice;
import frc.robot.util.akit.device.impl.TalonFXMotor;

public class WristIOTalonFX implements WristIO {
    final CANcoder wristCancoder =
            CANCoderDevice.configure(WristConfigs.WRIST_CANCODER_ID, Constants.CANIVORE_BUS)
                    .log("WristIO/WristCancoder")
                    .using(WristConfigs.wristEncoderConfigs)
                    .syncConfigs()
                    .getDevice();

    final TalonFX wristMotor =
            TalonFXMotor.configure(WristConfigs.WRIST_KRAKEN_ID, Constants.CANIVORE_BUS)
                    .log("WristIO/WristMotor")
                    .using(WristConfigs.wristMotorConfigs)
                    .usingFusedCANcoder(wristCancoder)
                    .syncConfigs()
                    .getDevice();
    private final StatusSignal<Angle> positionSignal = wristMotor.getPosition();
    private final PositionVoltage motorReq = new PositionVoltage(0);

    @Override
    public Angle getPosition() {
        return positionSignal.getValue();
    }

    @Override
    public void setPosition(Angle value) {
        wristMotor.setControl(motorReq.withPosition(value));
    }

    @Override
    public boolean isAtSetpoint(Angle setpoint) {
        return isAtSetpoint(setpoint, WristConfigs.WRIST_TOLERANCE);
    }

    @Override
    public boolean isAtSetpoint(Angle setpoint, Angle tolerance) {
        return getPosition().isNear(setpoint, tolerance);
    }
}
