package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.coral.CoralManipulatorState;
import java.util.Map;
import java.util.function.Supplier;

public class NewLEDSubsystem extends SubsystemBase {
    public AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    public ProgressBarPercents progressBarState;

    public enum ProgressBarPercents {
        ZERO(0),
        TWENTY(20),
        FORTY(40),
        SIXTY(60),
        EIGHTY(80),
        FULL(100);

        private final int progress;

        ProgressBarPercents(int progress) {
            this.progress = progress;
        }

        public int getProgress() {
            return progress;
        }
    }

    public NewLEDSubsystem() {
        ledStrip = new AddressableLED(Constants.LEDs.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDs.LED_COUNT);
        progressBarState = ProgressBarPercents.ZERO;
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
        setDefaultCommand(runPattern(Constants.LEDs.YETI_BLUE_SCROLLING));
        new Trigger(DriverStation::isAutonomousEnabled)
                .onTrue(runPattern(Constants.LEDs.AUTO_PATTERN));
        new Trigger(DriverStation::isDisabled)
                .whileTrue(runPattern(Constants.LEDs.PROGRESS_BAR_ZERO));
    }

    public void addProgress() {
        switch (progressBarState) {
            case ZERO -> progressBarState = ProgressBarPercents.TWENTY;
            case TWENTY -> progressBarState = ProgressBarPercents.FORTY;
            case FORTY -> progressBarState = ProgressBarPercents.SIXTY;
            case SIXTY -> progressBarState = ProgressBarPercents.EIGHTY;
            case EIGHTY -> progressBarState = ProgressBarPercents.FULL;
        }
        setProgress(progressBarState);
    }

    public void subtractProgress() {
        switch (progressBarState) {
            case FULL -> progressBarState = ProgressBarPercents.EIGHTY;
            case EIGHTY -> progressBarState = ProgressBarPercents.SIXTY;
            case SIXTY -> progressBarState = ProgressBarPercents.FORTY;
            case FORTY -> progressBarState = ProgressBarPercents.TWENTY;
            case TWENTY -> progressBarState = ProgressBarPercents.ZERO;
        }
        setProgress(progressBarState);
    }

    public void setProgress(ProgressBarPercents type) {
        switch (type) {
            case ZERO:
                runPattern(Constants.LEDs.PROGRESS_BAR_ZERO);
                progressBarState = ProgressBarPercents.ZERO;
                break;
            case TWENTY:
                runPattern(Constants.LEDs.PROGRESS_BAR_TWENTY);
                progressBarState = ProgressBarPercents.TWENTY;
                break;
            case FORTY:
                runPattern(Constants.LEDs.PROGRESS_BAR_FORTY);
                progressBarState = ProgressBarPercents.FORTY;
                break;
            case SIXTY:
                runPattern(Constants.LEDs.PROGRESS_BAR_SIXTY);
                progressBarState = ProgressBarPercents.SIXTY;
                break;
            case EIGHTY:
                runPattern(Constants.LEDs.PROGRESS_BAR_EIGHTY);
                progressBarState = ProgressBarPercents.EIGHTY;
                break;
            case FULL:
                runPattern(Constants.LEDs.PROGRESS_BAR_FULL);
                progressBarState = ProgressBarPercents.FULL;
                break;
        }
    }

    @Override
    public void periodic() {
        ledStrip.setData(ledBuffer);
        SmartDashboard.putString("Progress Bar", progressBarState.toString());
    }

    public Command runPattern(LEDPattern pattern) {
        return Commands.print("hi")
                .andThen(run(() -> pattern.applyTo(ledBuffer)))
                .repeatedly()
                .ignoringDisable(true);
    }

    public Command selectAnimationCommand(Supplier<CoralManipulatorState> getCMS) {
        return new SelectCommand<CoralManipulatorState>(
                Map.ofEntries(
                        Map.entry(
                                CoralManipulatorState.DISABLED,
                                runPattern(Constants.LEDs.PROGRESS_BAR_ZERO)),
                        Map.entry(
                                CoralManipulatorState.L1,
                                runPattern(Constants.LEDs.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.L2,
                                runPattern(Constants.LEDs.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.L3,
                                runPattern(Constants.LEDs.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.L4,
                                runPattern(Constants.LEDs.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.SCORE_L1,
                                runPattern(Constants.LEDs.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.SCORE_L2,
                                runPattern(Constants.LEDs.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.SCORE_L3,
                                runPattern(Constants.LEDs.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.SCORE_L4,
                                runPattern(Constants.LEDs.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.HP_INTAKE,
                                runPattern(Constants.LEDs.WHITE_BLINK)),
                        Map.entry(
                                CoralManipulatorState.GROUND_INTAKE,
                                runPattern(Constants.LEDs.WHITE_BLINK)),
                        Map.entry(
                                CoralManipulatorState.STOWED,
                                runPattern(Constants.LEDs.WHITE)
                                        .withTimeout(2)
                                        .andThen(runPattern(Constants.LEDs.YETI_BLUE_SCROLLING))),
                        Map.entry(
                                CoralManipulatorState.ALGAEHIGH,
                                runPattern(Constants.LEDs.ALGAE_COLOR_PATTERN)),
                        Map.entry(
                                CoralManipulatorState.CLIMB,
                                runPattern(Constants.LEDs.SCROLLING_RAINBOW))),
                () -> {
                    CoralManipulatorState state = getCMS.get();
                    System.out.println("Current CoralManipulatorState: " + state);
                    return state;
                });
    }
}
