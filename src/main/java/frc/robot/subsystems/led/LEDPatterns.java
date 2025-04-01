package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LEDConstants;

public enum LEDPatterns {
    YETI_BLUE_PATTERN(LEDPattern.solid(LEDConstants.YETI_BLUE)),
    YETI_BLUE_RSL_BLINK(YETI_BLUE_PATTERN.pattern.synchronizedBlink(RobotController::getRSLState)),
    WHITE(LEDPattern.solid(Color.kWhite)),
    YETI_BLUE_SCROLLING(
            LEDPattern.gradient(
                            LEDPattern.GradientType.kContinuous,
                            Color.kWhite,
                            LEDConstants.YETI_BLUE)
                    .scrollAtAbsoluteSpeed(
                            Centimeters.per(Second).of(-30), LEDConstants.LED_SPACING)),
    WHITE_BLINK(WHITE.pattern.blink(Seconds.of(0.3))),
    RAINBOW(LEDPattern.rainbow(255, 128)),
    ALGAE_COLOR_PATTERN(LEDPattern.solid(LEDConstants.ALGAE_COLOR)),
    AUTO_ALIGN(LEDPattern.solid(Color.kGold)),
    AUTO_PATTERN(LEDPattern.solid(Color.kSilver).breathe(Seconds.of(3))),
    NICK_MODE(LEDPattern.solid(LEDConstants.NICK_ORANGE).blink(Seconds.of(1))),
    SCROLLING_RAINBOW(
            RAINBOW.pattern.scrollAtAbsoluteSpeed(
                    MetersPerSecond.of(-1), LEDConstants.LED_SPACING));

    public final LEDPattern pattern;

    LEDPatterns(LEDPattern pattern) {
        this.pattern = pattern;
    }
}
