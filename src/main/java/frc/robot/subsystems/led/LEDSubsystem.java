package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LEDConfigs.*;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class LEDSubsystem extends SubsystemBase {
    public final CANdle candle = new CANdle(0, Constants.RIO_BUS);
    private Animation toAnimate = null;
    public int ledCount = 36;
    public int ledOffset = 8;
    private CANdleConfiguration configAll;
    public ProgressBar progressBar;
    private Events event;

    public enum Events {
        NICK,
        CORALINTAKE,
        CORALSTOWED,
        ALGAEINTAKE,
        IDLETELEOP,
        OFF,
        SINGLEFADE,
        LOS,
        PROGRESSBAR,
        CLEARPROGRESS
    }

    public LEDSubsystem() {
        configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = CANdle.LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = CANdle.VBatOutputMode.On;
        candle.configAllSettings(configAll, 100);
        progressBar = new ProgressBar(this);
    }

    public void clearAnimation() {
        candle.clearAnimation(0);
    }

    public void setAnimation(Events animation) {
        switch (animation) {
            case NICK:
                event = Events.NICK;
                toAnimate = new StrobeAnimation(255, 0, 25, 0, 4, ledCount);
                break;
            case ALGAEINTAKE:
                event = Events.ALGAEINTAKE;
                toAnimate =
                        new LarsonAnimation(
                                79, 165, 181, 0, 0.3, ledCount, BounceMode.Front, 3, ledOffset);
                break;
            case CORALINTAKE:
                event = Events.CORALINTAKE;
                toAnimate =
                        new LarsonAnimation(
                                255, 255, 255, 255, 0.3, ledCount, BounceMode.Front, 7, ledOffset);
                break;
            case CORALSTOWED:
                event = Events.CORALSTOWED;
                toAnimate = new StrobeAnimation(255, 255, 255, 255, 3, ledCount);
                break;
            case IDLETELEOP:
                event = Events.IDLETELEOP;
                toAnimate =
                        isRedAlliance()
                                ? new LarsonAnimation(
                                        255, 0, 0, 0, 0.4, ledCount, BounceMode.Front, 3, ledOffset)
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
                break;
            case OFF:
                event = Events.OFF;
                toAnimate = new StrobeAnimation(0, 0, 0, 0, 4, ledCount);
                break;
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
                if (DriverStation.isDisabled()) {
                    event = Events.PROGRESSBAR;
                }
                break;
            case CLEARPROGRESS:
                candle.setLEDs(0, 0, 0);
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
        candle.animate(toAnimate);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() && event == Events.PROGRESSBAR) {
            progressBar.setProgress(progressBar.progressBarState);
        }
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance()
                .filter(value -> value == DriverStation.Alliance.Red)
                .isPresent();
    }
}
