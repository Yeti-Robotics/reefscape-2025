package frc.robot.subsystems.coral.grabber.io;

import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.util.akit.device.can.cancolor.CANColorDevice;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

public class GrabberSetpointIOTalonFX implements GrabberSetpointIO {
    private final Canandcolor clawSwitch =
            CANColorDevice.configure(GrabberConfig.CLAW_ID)
                    .log("GrabberIO/ColorSensor")
                    .getDevice();
    private final TalonFX grabberMotor =
            TalonFXDevice.configure(GrabberConfig.CLAW_ID, Constants.RIO_BUS)
                    .log("GrabberIO/GrabberMotor")
                    .using(GrabberConfig.coralMotorConfig)
                    .syncConfigs()
                    .getDevice();

    @Override
    public void toSetpoint(GrabberState setpoint) {
        grabberMotor.set(setpoint.getSetpoint());
    }

    @Override
    public boolean isAtSetpoint(Double setpoint) {
        return isAtSetpoint(setpoint, 0.0);
    }

    @Override
    public boolean isAtSetpoint(Double setpoint, Double tolerance) {
        return MathUtil.isNear(setpoint, grabberMotor.get(), tolerance);
    }

    @Override
    public boolean hasCoral() {
        return clawSwitch.getProximity() < 0.05;
    }
}
