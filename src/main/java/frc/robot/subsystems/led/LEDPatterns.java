package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

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
    RED_ALLIANCE_PATTERN(LEDPattern.solid(Color.kRed)),
    RED_ALLIANCE_RSL_BLINK(
            RED_ALLIANCE_PATTERN.pattern.synchronizedBlink(RobotController::getRSLState)),
    NICK_MODE(
            LEDPattern.solid(LEDConstants.NICK_ORANGE)
                    .synchronizedBlink(RobotController::getRSLState)),
    SCROLLING_RAINBOW(
            RAINBOW.pattern.scrollAtAbsoluteSpeed(
                    MetersPerSecond.of(-1), LEDConstants.LED_SPACING)),

    FADING_BLUE_SCROLL(
            LEDPattern.gradient(
                            LEDPattern.GradientType.kContinuous,
                            LEDConstants.CRISP_WHITE,
                            LEDConstants.YETI_BLUE)
                    .scrollAtAbsoluteSpeed(
                            Centimeters.per(Second).of(-30), LEDConstants.LED_SPACING)
                    .mask(
                            LEDPattern.gradient(
                                    LEDPattern.GradientType.kDiscontinuous,
                                    LEDConstants.CRISP_WHITE,
                                    Color.kBlack))
                    .atBrightness(Percent.of(100)));

    public final LEDPattern pattern;

    LEDPatterns(LEDPattern pattern) {
        this.pattern = pattern;
    }
}
