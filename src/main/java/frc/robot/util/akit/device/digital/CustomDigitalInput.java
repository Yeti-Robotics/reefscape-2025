package frc.robot.util.akit.device.digital;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class CustomDigitalInput extends DigitalInput {
    private final Debouncer debouncer;
    private final boolean isInverted;

    /**
     * Create an instance of a Digital Input class. Creates a digital input given a channel.
     *
     * @param channel the DIO channel for the digital input 0-9 are on-board, 10-25 are on the MXP
     */
    public CustomDigitalInput(int channel, DigitalInputConfig config) {
        super(channel);
        debouncer = new Debouncer(config.debounce, config.debounceType);
        this.isInverted = config.isInverted;
    }

    @Override
    public boolean get() {
        return debouncer.calculate(isInverted ^ super.get());
    }
}
