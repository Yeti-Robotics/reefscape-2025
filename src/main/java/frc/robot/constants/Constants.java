// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int PRIMARY_XBOX_CONTROLLER_PORT = 0;
    public static final String CANIVORE_BUS = "canivoreBus";
    public static final String RIO_BUS = "rio";
    public static final double ZERO_TOLERANCE = 0.005;
    public static final int SECONDARY_XBOX_CONTROLLER_PORT = 1;

    public static final class LEDs {
        public static final int LED_STRIP_PORT = 0;
        public static final int LED_COUNT = 36;
        public static final Distance LED_SPACING = Meters.of(1 / 65.0);
        public static final Color YETI_BLUE = new Color(84, 182, 229);
        public static final Color ALGAE_COLOR = new Color(79, 181, 165);
        public static final Color NICK_ORANGE = new Color(240, 119, 99);
    }
}
