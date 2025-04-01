package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.coral.CoralManipulatorState;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class NewLEDSubsystem extends SubsystemBase {
    public AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private static final int PROGRESS_PARTS = 5;
    private static final double PROGRESS_INCREMENT = 1.0 / PROGRESS_PARTS;
    private double currentProgress = 0.0;

    public NewLEDSubsystem() {
        ledStrip = new AddressableLED(LEDConstants.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
        setDefaultCommand(run(this::updateProgress).ignoringDisable(true));
        new Trigger(DriverStation::isAutonomousEnabled)
                .onTrue(runPattern(LEDPatterns.AUTO_PATTERN));
        new Trigger(DriverStation::isTeleopEnabled)
                .onTrue(runPattern(LEDPatterns.YETI_BLUE_SCROLLING));
        new Trigger(DriverStation::isDisabled)
                .whileTrue(run(this::updateProgress).ignoringDisable(true));
    }

    public void progressIncrement(boolean positive) {
        currentProgress += positive ? PROGRESS_INCREMENT : -PROGRESS_INCREMENT;
        currentProgress = MathUtil.clamp(currentProgress, 0.0, 1.0);
    }

    public void addProgress() {
        progressIncrement(true);
        updateProgress();
    }

    public void subtractProgress() {
        progressIncrement(false);
        updateProgress();
    }

    public void updateProgress() {
        if (DriverStation.isDisabled()) {
            LEDPattern updatedPattern =
                    LEDPattern.solid(Color.kLimeGreen)
                            .mask(LEDPattern.progressMaskLayer(() -> currentProgress).reversed());
            run(() -> updatedPattern.applyTo(ledBuffer)).ignoringDisable(true).schedule();
        }
    }

    @Override
    public void periodic() {
        ledStrip.setData(ledBuffer);
        SmartDashboard.putNumber("Progress Bar", currentProgress);
    }

    public Command runPattern(LEDPatterns pattern) {
        return run(() -> pattern.pattern.applyTo(ledBuffer)).repeatedly().ignoringDisable(true);
    }


    public Command selectAnimationCommand(Supplier<CoralManipulatorState> getCMS) {
        EnumMap<CoralManipulatorState, Command> animationCommands = new EnumMap<>(CoralManipulatorState.class);
        animationCommands.put(CoralManipulatorState.DISABLED, runOnce(this::updateProgress));
        Command yetiBlueScrolling = runPattern(LEDPatterns.YETI_BLUE_SCROLLING);
        for (CoralManipulatorState state : new CoralManipulatorState[] {
                CoralManipulatorState.L1, CoralManipulatorState.L2, CoralManipulatorState.L3, CoralManipulatorState.L4,
                CoralManipulatorState.SCORE_L1, CoralManipulatorState.SCORE_L2, CoralManipulatorState.SCORE_L3, CoralManipulatorState.SCORE_L4}) {
            animationCommands.put(state, yetiBlueScrolling);
        }
        Command whiteBlink = runPattern(LEDPatterns.WHITE_BLINK);
        animationCommands.put(CoralManipulatorState.HP_INTAKE, whiteBlink);
        animationCommands.put(CoralManipulatorState.GROUND_INTAKE, whiteBlink);

        animationCommands.put(CoralManipulatorState.STOWED,
                runPattern(LEDPatterns.WHITE)
                        .withTimeout(2)
                        .andThen(runPattern(LEDPatterns.YETI_BLUE_SCROLLING)));

        animationCommands.put(CoralManipulatorState.ALGAEHIGH, runPattern(LEDPatterns.ALGAE_COLOR_PATTERN));
        animationCommands.put(CoralManipulatorState.CLIMB, runPattern(LEDPatterns.SCROLLING_RAINBOW));

        return new SelectCommand<>(animationCommands, () -> {
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
