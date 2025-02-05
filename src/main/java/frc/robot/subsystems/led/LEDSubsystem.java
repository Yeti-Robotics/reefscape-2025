package frc.robot.subsystems.led;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;

public class LEDSubsystem extends SubsystemBase {
    public final CANdle candle = new CANdle(0, Constants.RIO_BUS);
    private Animation toAnimate = null;
    public int ledCount = 39;
    public int ledOffset = 8;

    public enum Events {
        NICK,
        CORALINTAKE,
        CORALSTOWED,
        ALGAEINTAKE,
        IDLETELEOP,
        PROGRESSBAR,
        FLAME,
        OFF,
        LOS;
    }

    public LEDSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.On;
        candle.configAllSettings(configAll, 100);
    }

    public void setAnimation(Events animation) {
        switch (animation) {
            case NICK:
                toAnimate = new StrobeAnimation(255, 0, 25, 0, 4, ledCount);
                candle.animate(toAnimate);
                break;
            case ALGAEINTAKE:
                toAnimate =
                        new LarsonAnimation(
                                79, 165, 181, 0, 0.3, ledCount, BounceMode.Front, 3, ledOffset);
                candle.animate(toAnimate);
                break;
            case CORALINTAKE:
                toAnimate =
                        new LarsonAnimation(
                                255, 255, 255, 255, 0.3, ledCount, BounceMode.Front, 7, ledOffset);
                candle.animate(toAnimate);
                break;
            case CORALSTOWED:
                toAnimate = new StrobeAnimation(255, 255, 255, 255, 3, ledCount);
                candle.animate(toAnimate);
                break;
            case IDLETELEOP:
                if (Robot.isRedAlliance()) {
                    toAnimate =
                            new LarsonAnimation(
                                    255, 0, 0, 0, 0.3, ledCount, BounceMode.Front, 3, ledOffset);
                    candle.animate(toAnimate);
                } else {
                    toAnimate =
                            new LarsonAnimation(
                                    84, 229, 182, 0, 0.3, ledCount, BounceMode.Front, 3, ledOffset);
                    candle.animate(toAnimate);
                }
                break;
            case PROGRESSBAR:
                ProgressBar progressBar = new ProgressBar(this);
                progressBar.setProgress(ProgressBar.ProgressBarPercents.FULL);
                break;
            case FLAME:
                toAnimate = new FireAnimation(1, 0.2, 15, 0.5, 0.2, false, 8);
                candle.animate(toAnimate);
                break;
            case OFF:
                toAnimate = new StrobeAnimation(0, 0, 0, 0, 4, ledCount);
                candle.animate(toAnimate);
                break;
            case LOS:
                if (Robot.isRedAlliance()) {
                    toAnimate = new StrobeAnimation(255, 0, 0, 0, 0.2, ledCount);
                    candle.animate(toAnimate);
                } else {
                    toAnimate = new StrobeAnimation(84, 229, 182, 0, 0.2, ledCount);
                    candle.animate(toAnimate);
                }
                break;
        }
    }
}
