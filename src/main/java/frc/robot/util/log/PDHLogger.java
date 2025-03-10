package frc.robot.util.log;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.PowerDistribution;

@CustomLoggerFor(PowerDistribution.class)
public class PDHLogger extends ClassSpecificLogger<PowerDistribution> {
    public PDHLogger() {
        super(PowerDistribution.class);
    }

    @Override
    protected void update(EpilogueBackend backend, PowerDistribution pdh) {
        backend.log("getTotalCurrent", pdh.getTotalCurrent());
        backend.log("getAllCurrents", pdh.getAllCurrents());
        backend.log("getTemperature", pdh.getTemperature());
        backend.log("getVoltage", pdh.getVoltage());
        backend.log("getTotalEnergy", pdh.getTotalEnergy());
    }
}
