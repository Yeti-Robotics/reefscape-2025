package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.LEDConstants;
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
        ledStrip = new AddressableLED(LEDConstants.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        progressBarState = ProgressBarPercents.ZERO;
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
        setDefaultCommand(run(() -> setProgress(progressBarState)).ignoringDisable(true));
        new Trigger(DriverStation::isAutonomousEnabled)
                .onTrue(runPattern(LEDPatterns.AUTO_PATTERN));
        new Trigger(DriverStation::isTeleopEnabled)
                .onTrue(runPattern(LEDPatterns.YETI_BLUE_SCROLLING));
        System.out.println(progressBarState.toString());
        new Trigger(DriverStation::isDisabled)
                .whileTrue(run(() -> setProgress(progressBarState)).ignoringDisable(true));
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
        if (DriverStation.isDisabled()) {
            switch (type) {
                case ZERO:
                    progressBarState = ProgressBarPercents.ZERO;
                    runPattern(LEDPatterns.PROGRESS_BAR_ZERO).schedule();
                    break;
                case TWENTY:
                    progressBarState = ProgressBarPercents.TWENTY;
                    runPattern(LEDPatterns.PROGRESS_BAR_TWENTY).schedule();
                    break;
                case FORTY:
                    progressBarState = ProgressBarPercents.FORTY;
                    runPattern(LEDPatterns.PROGRESS_BAR_FORTY).schedule();
                    break;
                case SIXTY:
                    progressBarState = ProgressBarPercents.SIXTY;
                    runPattern(LEDPatterns.PROGRESS_BAR_SIXTY).schedule();
                    break;
                case EIGHTY:
                    progressBarState = ProgressBarPercents.EIGHTY;
                    runPattern(LEDPatterns.PROGRESS_BAR_EIGHTY).schedule();
                    break;
                case FULL:
                    progressBarState = ProgressBarPercents.FULL;
                    runPattern(LEDPatterns.PROGRESS_BAR_FULL).schedule();
                    break;
            }
        }
    }

    @Override
    public void periodic() {
        ledStrip.setData(ledBuffer);
        SmartDashboard.putNumber("Progress Bar", progressBarState.getProgress());
    }

    public Command runPattern(LEDPatterns pattern) {
        return Commands.print("hi")
                .andThen(run(() -> pattern.pattern.applyTo(ledBuffer)))
                .repeatedly()
                .ignoringDisable(true);
    }

    public Command selectAnimationCommand(Supplier<CoralManipulatorState> getCMS) {
        return new SelectCommand<CoralManipulatorState>(
                Map.ofEntries(
                        Map.entry(
                                CoralManipulatorState.DISABLED,
                                runPattern(LEDPatterns.PROGRESS_BAR_ZERO)),
                        Map.entry(
                                CoralManipulatorState.L1,
                                runPattern(LEDPatterns.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.L2,
                                runPattern(LEDPatterns.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.L3,
                                runPattern(LEDPatterns.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.L4,
                                runPattern(LEDPatterns.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.SCORE_L1,
                                runPattern(LEDPatterns.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.SCORE_L2,
                                runPattern(LEDPatterns.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.SCORE_L3,
                                runPattern(LEDPatterns.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.SCORE_L4,
                                runPattern(LEDPatterns.YETI_BLUE_SCROLLING)),
                        Map.entry(
                                CoralManipulatorState.HP_INTAKE,
                                runPattern(LEDPatterns.WHITE_BLINK)),
                        Map.entry(
                                CoralManipulatorState.GROUND_INTAKE,
                                runPattern(LEDPatterns.WHITE_BLINK)),
                        Map.entry(
                                CoralManipulatorState.STOWED,
                                runPattern(LEDPatterns.WHITE)
                                        .withTimeout(2)
                                        .andThen(runPattern(LEDPatterns.YETI_BLUE_SCROLLING))),
                        Map.entry(
                                CoralManipulatorState.ALGAEHIGH,
                                runPattern(LEDPatterns.ALGAE_COLOR_PATTERN)),
                        Map.entry(
                                CoralManipulatorState.CLIMB,
                                runPattern(LEDPatterns.SCROLLING_RAINBOW))),
                () -> {
                    CoralManipulatorState state = getCMS.get();
                    System.out.println("Current CoralManipulatorState: " + state);
                    return state;
                });
    }

    private static boolean isRedAlliance() {
        return DriverStation.getAlliance()
                .filter(value -> value == DriverStation.Alliance.Red)
                .isPresent();
    }
}
