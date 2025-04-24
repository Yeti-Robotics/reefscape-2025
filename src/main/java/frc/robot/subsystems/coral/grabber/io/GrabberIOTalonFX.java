package frc.robot.subsystems.coral.grabber.io;

import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import frc.robot.constants.Constants;
import frc.robot.util.akit.device.can.cancolor.CANColorDevice;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

public class GrabberIOTalonFX implements GrabberIO {
    private final Canandcolor clawSwitch = CANColorDevice.configure(GrabberConfig.CLAW_ID)
            .log("GrabberIO/ColorSensor")
            .getDevice();

    final TalonFX grabberMotor = TalonFXDevice.configure(GrabberConfig.CLAW_ID, Constants.RIO_BUS)
            .log("GrabberIO/GrabberMotor")
            .withConfig(GrabberConfig.coralMotorConfig)
            .syncConfigs()
            .getDevice();

    @Override
    public Double getState() {
        return grabberMotor.get();
    }

    @Override
    public void setState(Double value) {
        grabberMotor.set(value);
    }

    @Override
    public boolean hasCoral() {
        return clawSwitch.getProximity() < 0.05;
    }
}
