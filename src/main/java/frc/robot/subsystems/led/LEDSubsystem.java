package frc.robot.subsystems.led;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import java.util.Map;

public class LEDSubsystem extends SubsystemBase {
    public final CANdle candle = new CANdle(0, Constants.RIO_BUS);
    public final AddressableLED led = new AddressableLED(0);
    public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(36);
    private Animation toAnimate = null;
    private Events currentAnimation;

    public enum Events {
        NICK,
        CORALINTAKE,
        CORALSTOWED,
        ALGAEINTAKE,
        IDLETELEOP,
        PROGRESSBAR;
    }

    public LEDSubsystem() {
        led.setLength(ledBuffer.getLength());
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.On;
        candle.configAllSettings(configAll, 1);
    }

    public void changeAnimation(Events animation) {
        int ledCount = 39;
        int ledOffset = 8;
        switch (animation) {
            case NICK:
                toAnimate = new StrobeAnimation(255, 0, 25, 0, 4, ledCount);
                break;
            case ALGAEINTAKE:
                toAnimate =
                        new LarsonAnimation(
                                79, 165, 181, 0, 0.3, ledCount, BounceMode.Front, 3, ledOffset);
                break;
            case CORALINTAKE:
                toAnimate =
                        new LarsonAnimation(
                                255, 255, 255, 255, 0.3, ledCount, BounceMode.Front, 7, ledOffset);
                break;
            case CORALSTOWED:
                toAnimate = new StrobeAnimation(255, 255, 255, 255, 3, ledCount);
                break;
            case IDLETELEOP:
                if (Robot.isRedAlliance()) {
                    toAnimate =
                            new LarsonAnimation(
                                    255, 0, 0, 0, 0.3, ledCount, BounceMode.Front, 3, ledOffset);
                } else {
                    toAnimate =
                            new LarsonAnimation(
                                    84, 229, 182, 0, 0.3, ledCount, BounceMode.Front, 3, ledOffset);
                }
                break;
            case PROGRESSBAR:
                LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kGreen));
                steps.applyTo(ledBuffer);
                led.setData(ledBuffer);
                break;
        }
    }

    @Override
    public void periodic() {
        changeAnimation(Events.NICK);
        candle.animate(toAnimate);
    }
}
