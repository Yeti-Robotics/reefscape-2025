package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.CoralManipulatorState;
import java.util.EnumMap;
import java.util.function.Supplier;

public class LEDSubsystem extends SubsystemBase {
    public AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftBufferView;
    private final AddressableLEDBufferView rightBufferView;
    private double currentProgress = 0.0;

    public LEDSubsystem() {
        ledStrip = new AddressableLED(LEDConstants.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        leftBufferView = ledBuffer.createView(0, 35);
        rightBufferView = ledBuffer.createView(36, 71);
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
        setDefaultCommand(run(this::updateProgress).ignoringDisable(true));
        new Trigger(DriverStation::isAutonomousEnabled)
                .onTrue(runPattern(LEDPatterns.AUTO_PATTERN));
        new Trigger(DriverStation::isTeleopEnabled)
                .onTrue(runPattern(LEDPatterns.YETI_BLUE_RSL_BLINK));
        new Trigger(DriverStation::isDisabled)
                .whileTrue(run(this::updateProgress).ignoringDisable(true));
    }

    public void progressIncrement(boolean positive) {
        currentProgress +=
                positive ? LEDConstants.PROGRESS_INCREMENT : -LEDConstants.PROGRESS_INCREMENT;
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
                    LEDPattern.solid(new Color(0, 0, 255))
                            .mask(LEDPattern.progressMaskLayer(() -> currentProgress));
            run(() -> updatedPattern.applyTo(leftBufferView))
                    .andThen(() -> updatedPattern.applyTo(rightBufferView))
                    .ignoringDisable(true)
                    .schedule();
        }
    }

    @Override
    public void periodic() {
        ledStrip.setData(ledBuffer);
        SmartDashboard.putNumber("Progress Bar", currentProgress);
        SmartDashboard.putBoolean("Is Battery Good?", RobotController.getBatteryVoltage() > 12.5);
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    }

    public Command runPattern(LEDPatterns pattern) {
        return run(() -> pattern.pattern.applyTo(leftBufferView))
                .andThen(() -> pattern.pattern.applyTo(rightBufferView))
                .repeatedly()
                .ignoringDisable(true);
    }

    public Command selectAnimationCommand(Supplier<CoralManipulatorState> getCMS) {
        EnumMap<CoralManipulatorState, Command> animationCommands =
                new EnumMap<>(CoralManipulatorState.class);
        animationCommands.put(CoralManipulatorState.DISABLED, runOnce(this::updateProgress));
        for (CoralManipulatorState state :
                new CoralManipulatorState[] {
                    CoralManipulatorState.L1, CoralManipulatorState.L2, CoralManipulatorState.L3,
                            CoralManipulatorState.L4,
                    CoralManipulatorState.SCORE_L1, CoralManipulatorState.SCORE_L2,
                            CoralManipulatorState.SCORE_L3, CoralManipulatorState.SCORE_L4
                }) {
            animationCommands.put(state, runPattern(LEDPatterns.YETI_BLUE_SCROLLING));
        }
        animationCommands.put(CoralManipulatorState.HP_INTAKE, runPattern(LEDPatterns.WHITE_BLINK));
        animationCommands.put(
                CoralManipulatorState.GROUND_INTAKE, runPattern(LEDPatterns.WHITE_BLINK));

        animationCommands.put(
                CoralManipulatorState.STOWED,
                runPattern(LEDPatterns.WHITE)
                        .withTimeout(2)
                        .andThen(runPattern(LEDPatterns.YETI_BLUE_RSL_BLINK)));

        animationCommands.put(
                CoralManipulatorState.ALGAEHIGH, runPattern(LEDPatterns.ALGAE_COLOR_PATTERN));
        animationCommands.put(
                CoralManipulatorState.CLIMB, runPattern(LEDPatterns.SCROLLING_RAINBOW));

        return new SelectCommand<>(
                animationCommands,
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
