package frc.robot.subsystems.coral.grabber.io;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;
import frc.robot.subsystems.coral.grabber.io.sensor.CoralSensor;
import frc.robot.util.akit.device.can.CANUtil;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

import java.util.function.Supplier;

public class GrabberIOTalonFX implements GrabberIO {
    private final TalonFXDevice grabberMotor = TalonFXDevice.configure(GrabberConfig.CLAW_ID, Constants.RIO_BUS)
            .log("Grabber/Motor")
            .withConfig(GrabberConfig.coralMotorConfig)
            .syncConfigs();

    private final Supplier<Double> dutyCycleSupplier = grabberMotor.mapStatusSignalWithInputs(CANUtil.TALON_MAX_UPDATE_HZ, TalonFX::getDutyCycle, inputs -> inputs.motorInputs.dutyCycle);
    private final CoralSensor sensor = CoralSensor.createCoralSensor();

    @Override
    public Double getState() {
        return dutyCycleSupplier.get();
    }

    @Override
    public void setState(Double value) {
        grabberMotor.getDevice().set(value);
    }

    @Override
    public boolean hasCoral() {
        return sensor.hasCoral();
    }
}
