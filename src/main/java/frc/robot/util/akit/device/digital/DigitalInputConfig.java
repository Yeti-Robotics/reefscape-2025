package frc.robot.util.akit.device.digital;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.filter.Debouncer;

public class DigitalInputConfig {
    public SimDevice device = null;
    public double debounce = 0.0;
    public Debouncer.DebounceType debounceType = Debouncer.DebounceType.kRising;
    public boolean isInverted = false;

    DigitalInputConfig() {}
}
