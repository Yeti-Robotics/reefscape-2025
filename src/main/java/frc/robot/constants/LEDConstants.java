package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    public static final int LED_STRIP_PORT =
            0; // TODO: update with the real values from the electrical kids
    public static final int LED_COUNT = 36;
    public static final Distance LED_SPACING = Meters.of(1 / 65.0);
    public static final Color YETI_BLUE = new Color(84, 182, 229);
    public static final Color ALGAE_COLOR = new Color(79, 181, 165);
    public static final Color NICK_ORANGE = new Color(240, 119, 99);
}
