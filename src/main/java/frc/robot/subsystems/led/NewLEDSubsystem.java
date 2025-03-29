package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.coral.CoralManipulatorState;
import java.util.Map;
import java.util.function.Supplier;

public class NewLEDSubsystem extends SubsystemBase {
    public final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    public NewLEDSubsystem() {
        ledStrip = new AddressableLED(Constants.LEDs.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDs.LED_COUNT);
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
        setDefaultCommand(runPattern(Constants.LEDs.YETI_BLUE_SCROLLING));
    }

    @Override
    public void periodic() {
        ledStrip.setData(ledBuffer);
    }

    private Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(ledBuffer));
    }

    public Command selectAnimationCommand(Supplier<CoralManipulatorState> getCMS) {
        return new SelectCommand<CoralManipulatorState>(
                Map.ofEntries(
                        Map.entry(
                                CoralManipulatorState.SCORE_L1,
                                runPattern(Constants.LEDs.SCROLLING_RAINBOW)
                                        .withTimeout(1)
                                        .andThen(runPattern(Constants.LEDs.YETI_BLUE_SCROLLING))),
                        Map.entry(
                                CoralManipulatorState.SCORE_L2,
                                runPattern(Constants.LEDs.SCROLLING_RAINBOW)
                                        .withTimeout(1)
                                        .andThen(runPattern(Constants.LEDs.YETI_BLUE_SCROLLING))),
                        Map.entry(
                                CoralManipulatorState.SCORE_L3,
                                runPattern(Constants.LEDs.SCROLLING_RAINBOW)
                                        .withTimeout(1)
                                        .andThen(runPattern(Constants.LEDs.YETI_BLUE_SCROLLING))),
                        Map.entry(
                                CoralManipulatorState.SCORE_L4,
                                runPattern(Constants.LEDs.SCROLLING_RAINBOW)
                                        .withTimeout(1)
                                        .andThen(runPattern(Constants.LEDs.YETI_BLUE_SCROLLING))),
                        Map.entry(
                                CoralManipulatorState.HP_INTAKE,
                                runPattern(Constants.LEDs.WHITE_BLINK)),
                        Map.entry(
                                CoralManipulatorState.GROUND_INTAKE,
                                runPattern(Constants.LEDs.WHITE_BLINK)),
                        Map.entry(CoralManipulatorState.STOWED, runPattern(Constants.LEDs.WHITE)),
                        Map.entry(
                                CoralManipulatorState.ALGAEHIGH,
                                runPattern(Constants.LEDs.YETI_BLUE_RSL_BLINK)
                        ),
                        Map.entry(
                                CoralManipulatorState.CLIMB,
                                runPattern(Constants.LEDs.SCROLLING_RAINBOW))),
                getCMS);
    }
}
