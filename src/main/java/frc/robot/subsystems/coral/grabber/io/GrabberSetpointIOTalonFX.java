package frc.robot.subsystems.coral.grabber.io;

import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
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

    // when wpilib removes this, probably just create a custom implementation
    private final MutDimensionless grabberMotorValue = Units.Value.mutable(0);

    @Override
    public Dimensionless getState() {
        return grabberMotorValue.mut_setMagnitude(grabberMotor.get());
    }

    @Override
    public void setState(Dimensionless value) {
        grabberMotor.set(value.magnitude());
    }

    @Override
    public boolean hasCoral() {
        return clawSwitch.getProximity() < 0.05;
    }
}
