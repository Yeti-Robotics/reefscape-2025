package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LEDConfigs.*;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class LEDSubsystem extends SubsystemBase {
    public final CANdle candle = new CANdle(0, Constants.RIO_BUS);
    private Animation toAnimate = null;
    public int ledCount = 36;
    public int ledOffset = 8;
    private CANdleConfiguration configAll;

    public enum Events {
        NICK,
        CORALINTAKE,
        CORALSTOWED,
        ALGAEINTAKE,
        IDLETELEOP,
        OFF,
        SINGLEFADE,
        LOS;
    }

    public LEDSubsystem() {
        configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = CANdle.LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = CANdle.VBatOutputMode.On;
        candle.configAllSettings(configAll, 100);


    }

    public void clearAnimation() {
        candle.clearAnimation(0);
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
                candle.animate(toAnimate);
                break;
            case OFF:
                toAnimate = new StrobeAnimation(0, 0, 0, 0, 4, ledCount);
                candle.animate(toAnimate);
                break;
            case LOS:
                toAnimate =
                        isRedAlliance()
                                ? new StrobeAnimation(255, 0, 0, 0, 0.2, ledCount)
                                : new StrobeAnimation(84, 229, 182, 0, 0.2, ledCount);
                candle.animate(toAnimate);
                break;
            case SINGLEFADE:
                toAnimate = new SingleFadeAnimation(191, 255, 0, 0, 0.2, ledCount, ledOffset);
                candle.animate(toAnimate);
                break;
        }
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance()
                .filter(value -> value == DriverStation.Alliance.Red)
                .isPresent();
    }
}
