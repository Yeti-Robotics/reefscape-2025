package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.Constants;

public enum LEDPatterns {
    YETI_BLUE_PATTERN(LEDPattern.solid(Constants.LEDs.YETI_BLUE)),
    YETI_BLUE_RSL_BLINK(YETI_BLUE_PATTERN.pattern.synchronizedBlink(RobotController::getRSLState)),
    WHITE(LEDPattern.solid(Color.kWhite)),
    YETI_BLUE_SCROLLING(
            LEDPattern.gradient(
                            LEDPattern.GradientType.kContinuous,
                            Color.kWhite,
                            Constants.LEDs.YETI_BLUE)
                    .scrollAtAbsoluteSpeed(
                            Centimeters.per(Second).of(-30), Constants.LEDs.LED_SPACING)),
    WHITE_BLINK(WHITE.pattern.blink(Seconds.of(0.3))),
    RAINBOW(LEDPattern.rainbow(255, 128)),
    ALGAE_COLOR_PATTERN(LEDPattern.solid(Constants.LEDs.ALGAE_COLOR)),
    AUTO_ALIGN(LEDPattern.solid(Color.kGold)),
    AUTO_PATTERN(LEDPattern.solid(Color.kSilver).breathe(Seconds.of(3))),
    NICK_MODE(LEDPattern.solid(Constants.LEDs.NICK_ORANGE).blink(Seconds.of(1))),
    SCROLLING_RAINBOW(
            RAINBOW.pattern.scrollAtAbsoluteSpeed(
                    MetersPerSecond.of(-1), Constants.LEDs.LED_SPACING)),
    PROGRESS_BAR_ZERO(
            LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue)
                    .mask(LEDPattern.progressMaskLayer(() -> 0.0))
                    .reversed()),
    PROGRESS_BAR_TWENTY(
            LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue)
                    .mask(LEDPattern.progressMaskLayer(() -> 0.2))
                    .reversed()),
    PROGRESS_BAR_FORTY(
            LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue)
                    .mask(LEDPattern.progressMaskLayer(() -> 0.4))
                    .reversed()),
    PROGRESS_BAR_SIXTY(
            LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue)
                    .mask(LEDPattern.progressMaskLayer(() -> 0.6))
                    .reversed()),
    PROGRESS_BAR_EIGHTY(
            LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue)
                    .mask(LEDPattern.progressMaskLayer(() -> 0.8))
                    .reversed()),
    PROGRESS_BAR_FULL(
            LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue)
                    .mask(LEDPattern.progressMaskLayer(() -> 1.0))
                    .reversed());
    public final LEDPattern pattern;

    LEDPatterns(LEDPattern pattern) {
        this.pattern = pattern;
    }
}
