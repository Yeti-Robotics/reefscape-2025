package frc.robot.subsystems.led;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.Constants;

public class LEDSubsystem extends SubsystemBase {
    public final CANdle candle = new CANdle(0, Constants.RIO_BUS);
    public final AddressableLED led = new AddressableLED(0);
    public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(36);
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
        //        this.joystick = joystick;
        led.setLength(ledBuffer.getLength());
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.On;
        candle.configAllSettings(configAll, 1);
        //        System.out.println("Candle Output");
        //        System.out.println(candle.setLEDs(8, 255, 8, 8, 0, 14));
        //        candle.setLEDs(220, 0, 0, 22, 22, 17);
        new Trigger(DriverStation::isDisabled).onFalse(Commands.print("led off"));
    }

    public void changeAnimation(Events animation) {
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
                //                ProgressBar progressBar = new ProgressBar();
                //                progressBar.setProgress(ProgressBar.ProgressBarPercents.FIFTY);
                break;
            case FLAME:
                toAnimate = new FireAnimation(1, 0.2, 15, 0.5, 0.2, false, 8);
                break;
            case OFF:
                toAnimate = new StrobeAnimation(0, 0, 0, 0, 4, ledCount);
            case LOS:
                if (Robot.isRedAlliance()) {
                    toAnimate = new StrobeAnimation(255, 0, 0, 0, 0.2, ledCount);
                } else {
                    toAnimate = new StrobeAnimation(84, 229, 182, 0, 0.2, ledCount);
                }
        }
    }

    @Override
    public void periodic() {
        //        candle.setLEDs(
        //                (int) (joystick.getLeftTriggerAxis() * 255),
        //                (int) (joystick.getRightTriggerAxis() * 255),
        //                (int) (joystick.getLeftX() * 255));
        //        candle.setLEDs(100, 100, 210);
        if (DriverStation.isDisabled()) {
            candle.clearAnimation(0);
            candle.setLEDs(255, 0, 0);
        } else {
            changeAnimation(Events.IDLETELEOP);
            candle.animate(toAnimate);
        }
    }
}
