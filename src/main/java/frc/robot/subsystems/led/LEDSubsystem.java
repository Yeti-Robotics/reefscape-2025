package frc.robot.subsystems.led;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.coral.CoralManipulatorState;
import java.util.Map;
import java.util.function.Supplier;

public class LEDSubsystem extends SubsystemBase {
    public final CANdle candle = new CANdle(25, Constants.RIO_BUS);
    private Animation toAnimate = null;
    public int ledCount = 36;
    public int ledOffset = 8;
    private CANdleConfiguration configAll;
    public ProgressBar progressBar;
    private Events event;

    public LEDSubsystem() {
        configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = CANdle.LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = CANdle.VBatOutputMode.On;
        candle.configAllSettings(configAll, 100);
        progressBar = new ProgressBar(this);
        new Trigger(DriverStation::isDisabled)
                .onTrue(
                        runOnce(this::clearAnimation)
                                .andThen(runOnce(() -> setAnimation(Events.PROGRESSBAR))));
    }

    private void clearAnimation() {
        candle.clearAnimation(0);
    }

    public void setAnimation(Events animation) {
        clearAnimation();
        if (animation == null) {
            animation = Events.IDLETELEOP;
        }
        switch (animation) {
            case NICK:
                event = Events.NICK;
                candle.setLEDs(255, 0, 25, 0, ledOffset, ledCount);
//                toAnimate = new StrobeAnimation(255, 0, 25, 0, 4, ledCount);
                break;
            case ALGAEINTAKE:
                event = Events.ALGAEINTAKE;
                candle.setLEDs(79, 165, 181, 0, ledOffset, ledCount);
//                toAnimate =
//                        new LarsonAnimation(
//                                79, 165, 181, 0, 0.3, ledCount, BounceMode.Front, 3, ledOffset);
                break;
            case CORALINTAKE:
                event = Events.CORALINTAKE;
                candle.setLEDs(255, 255, 255, 255, ledOffset, ledCount);
//                toAnimate =
//                        new LarsonAnimation(
//                                255, 255, 255, 255, 0.3, ledCount, BounceMode.Front, 7, ledOffset);
                break;
            case CORALSTOWED:
                event = Events.CORALSTOWED;
                candle.setLEDs(255, 255, 255, 255, ledOffset, ledCount);
//                toAnimate = new StrobeAnimation(255, 255, 255, 255, 3, ledCount);
                break;
            case IDLETELEOP:
                event = Events.IDLETELEOP;
                candle.clearAnimation(0);
                candle.setLEDs(0, 0, 0, 0, ledOffset, ledCount);
                if (isRedAlliance()) {
                    candle.animate(new LarsonAnimation(
                            255, 0, 0, 0, 0.4, ledCount, BounceMode.Front, 3, ledOffset));
                } else {
                    candle.animate(new LarsonAnimation(
                            84, 229, 182, 0, 0.3, ledCount, BounceMode.Front, 3, ledOffset));
                }
//                toAnimate =
//                        isRedAlliance()
//                                ?
//                                : new LarsonAnimation(
//                                        84,
//                                        229,
//                                        182,
//                                        0,
//                                        0.3,
//                                        ledCount,
//                                        BounceMode.Front,
//                                        3,
//                                        ledOffset);
                break;
            case OFF:
                event = Events.OFF;
                toAnimate = new StrobeAnimation(0, 0, 0, 0, 4, ledCount);
                break;
            case LYNK_FIRE:
                event = Events.LYNK_FIRE;
                candle.clearAnimation(0);
                candle.setLEDs(0, 0, 0, 0, ledOffset, ledCount);
                candle.animate(new FireAnimation(1.0, 0.38, ledCount, 0.8, 0.2, false, ledOffset));
            case LOS:
                event = Events.LOS;
                toAnimate =
                        isRedAlliance()
                                ? new StrobeAnimation(255, 0, 0, 0, 0.2, ledCount)
                                : new StrobeAnimation(84, 229, 182, 0, 0.2, ledCount);
                break;
            case SINGLEFADE:
                event = Events.SINGLEFADE;
                toAnimate = new SingleFadeAnimation(191, 255, 0, 0, 0.2, ledCount, ledOffset);
                break;
            case PROGRESSBAR:
                event = Events.PROGRESSBAR;
                progressBar.setProgress(ProgressBar.ProgressBarPercents.ZERO);
                break;
            case CLEARPROGRESS:
                event = Events.CLEARPROGRESS;
                candle.setLEDs(0, 0, 0);
                break;
            case ELEVATORSCORE:
                event = Events.ELEVATORSCORE;
                clearAnimation();
                candle.setLEDs(0, 0, 0);
                candle.setLEDs(0, 0, 255, 0, ledOffset, ledCount);
//                toAnimate = new StrobeAnimation(0, 0, 255, 0, 4, ledCount);
                break;
            case ELEVATORMOVING:
                event = Events.ELEVATORMOVING;
                clearAnimation();
                candle.setLEDs(0, 0, 0);
                candle.setLEDs(255, 0, 255, 0, ledOffset, ledCount);
//                toAnimate = new StrobeAnimation(255, 0, 255, 0, 4, ledCount);
                break;
            default:
                if (toAnimate == null) {
                    toAnimate =
                            isRedAlliance()
                                    ? new LarsonAnimation(
                                            255,
                                            0,
                                            0,
                                            0,
                                            0.4,
                                            ledCount,
                                            BounceMode.Front,
                                            3,
                                            ledOffset)
                                    : new LarsonAnimation(
                                            84,
                                            229,
                                            182,
                                            0,
                                            0.3,
                                            ledCount,
                                            BounceMode.Front,
                                            3,
                                            ledOffset);
                    candle.animate(toAnimate);
                } else {
                    candle.animate(
                            isRedAlliance()
                                    ? new LarsonAnimation(
                                            255,
                                            0,
                                            0,
                                            0,
                                            0.4,
                                            ledCount,
                                            BounceMode.Front,
                                            3,
                                            ledOffset)
                                    : new LarsonAnimation(
                                            84,
                                            229,
                                            182,
                                            0,
                                            0.3,
                                            ledCount,
                                            BounceMode.Front,
                                            3,
                                            ledOffset));
                }
        }
//        candle.animate(toAnimate);
    }

    public Command selectAnimationCommand(Supplier<CoralManipulatorState> getCMS) {
        return new SelectCommand<CoralManipulatorState>(
                Map.ofEntries(
                        Map.entry(
                                CoralManipulatorState.DISABLED,
                                runOnce(() -> setAnimation(Events.PROGRESSBAR))),
                        Map.entry(
                                CoralManipulatorState.HP_INTAKE,
                                runOnce(() -> setAnimation(Events.LYNK_FIRE))),
                        Map.entry(
                                CoralManipulatorState.GROUND_INTAKE,
                                runOnce(() -> setAnimation(Events.CORALINTAKE))),
                        Map.entry(
                                CoralManipulatorState.L1,
                                runOnce(() -> setAnimation(Events.ELEVATORMOVING))),
                        Map.entry(
                                CoralManipulatorState.SCORE_L1,
                                runOnce(() -> setAnimation(Events.ELEVATORSCORE))),
                        Map.entry(
                                CoralManipulatorState.L2,
                                runOnce(() -> setAnimation(Events.ELEVATORMOVING))),
                        Map.entry(
                                CoralManipulatorState.SCORE_L2,
                                runOnce(() -> setAnimation(Events.ELEVATORSCORE))),
                        Map.entry(
                                CoralManipulatorState.L3,
                                runOnce(() -> setAnimation(Events.ELEVATORMOVING))),
                        Map.entry(
                                CoralManipulatorState.SCORE_L3,
                                runOnce(() -> setAnimation(Events.ELEVATORSCORE))),
                        Map.entry(
                                CoralManipulatorState.L4,
                                runOnce(() -> setAnimation(Events.ELEVATORMOVING))),
                        Map.entry(
                                CoralManipulatorState.SCORE_L4,
                                runOnce(() -> setAnimation(Events.ELEVATORSCORE))),
                        Map.entry(
                                CoralManipulatorState.IDLE,
                                runOnce(() -> setAnimation(Events.IDLETELEOP))),
                        Map.entry(
                                CoralManipulatorState.STOWED,
                                runOnce(() -> setAnimation(Events.CORALSTOWED)))),
                getCMS);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() && event == Events.PROGRESSBAR) {
            progressBar.setProgress(progressBar.progressBarState);
        }
        SmartDashboard.putString("LED State", event.toString());
    }

    private static boolean isRedAlliance() {
        return DriverStation.getAlliance()
                .filter(value -> value == DriverStation.Alliance.Red)
                .isPresent();
    }
}
