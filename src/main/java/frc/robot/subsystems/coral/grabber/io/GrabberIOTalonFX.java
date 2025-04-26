package frc.robot.subsystems.coral.grabber.io;

import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.akit.device.can.cancolor.CANColorDevice;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

public class GrabberIOTalonFX implements GrabberIO {
    private final Canandcolor clawSwitch = Robot.isSimulation()
            ? null // TODO: add simulation support for grabber switch
            : CANColorDevice.configure(GrabberConfig.GRABBER_CANANDCOLOR)
                    .log("Grabber/ColorSensor")
                    .getDevice();

    private final TalonFX grabberMotor = TalonFXDevice.configure(GrabberConfig.CLAW_ID, Constants.RIO_BUS)
            .log("Grabber/Motor")
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
        if (clawSwitch == null) return false;
        return clawSwitch.getProximity() < 0.05;
    }
}
