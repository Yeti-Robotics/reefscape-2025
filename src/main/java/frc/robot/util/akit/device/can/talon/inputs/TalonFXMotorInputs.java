package frc.robot.util.akit.device.can.talon.inputs;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TalonFXMotorInputs {
    public Voltage motorVoltage;
    public Current motorAmps;

    public Temperature motorTemperature;
}
