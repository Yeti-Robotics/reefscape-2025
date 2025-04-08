package frc.robot.subsystems.coral.grabber.io;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import frc.robot.constants.Constants;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

public class GrabberIOTalonFX implements GrabberIO {
    private final Canandcolor clawSwitch = new Canandcolor(GrabberConfig.GRABBER_CANANDCOLOR);
    private final TalonFX grabberMotor =
            TalonFXDevice.configure(GrabberConfig.CLAW_ID, Constants.RIO_BUS)
                    .log("GrabberIO/GrabberMotor")
                    .using(GrabberConfig.coralMotorConfig)
                    .syncConfigs()
                    .getDevice();

    private final DutyCycleOut motorReq = new DutyCycleOut(0);

    @Override
    public void toSetpoint(GrabberState setpoint) {
        grabberMotor.setControl(motorReq.withOutput(setpoint.getSetpoint()));
    }

    @Override
    public boolean isAtSetpoint(Double setpoint) {
        return isAtSetpoint(setpoint, 1.0);
    }

    @Override
    public boolean isAtSetpoint(Double setpoint, Double tolerance) {
        return grabberMotor.getAppliedControl().getName().equals("DutyCycleOut");
    }

    @Override
    public boolean hasCoral() {
        return clawSwitch.getProximity() < 0.05;
    }
}
