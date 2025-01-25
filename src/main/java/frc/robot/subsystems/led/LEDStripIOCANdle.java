package frc.robot.subsystems.led;

import com.ctre.phoenix.led.*;
import frc.robot.constants.Constants;

public class LEDStripIOCANdle implements LEDStripIO {

    private final CANdle candle;
    static final int candleID = 0;
    static final int numLed = 0;

    private final LarsonAnimation idleRedAnimation = new LarsonAnimation(255,0,0, 0, 0.03,
            numLed, LarsonAnimation.BounceMode.Center, 2);

    //Yeti blue fr fr
    private final LarsonAnimation idleBlueAnimation = new LarsonAnimation(84,182,229, 0, 0.03,
            numLed, LarsonAnimation.BounceMode.Center, 2);

    private final StrobeAnimation coralStowedAnimation = new StrobeAnimation(255, 255, 255,
            0, 0.03, numLed);

    private final LarsonAnimation coralIntakeAnimation = new LarsonAnimation(255,255,255,0,0.03,
            numLed,LarsonAnimation.BounceMode.Center, 5);

    private final LarsonAnimation algaeIntakeAnimation = new LarsonAnimation(79,181,165,0,0.03,
            numLed,LarsonAnimation.BounceMode.Center, 5);


    private final FireAnimation flameTune = new FireAnimation(0.8,0.75,
            numLed, 1, 0.3);

    //nickelodeon
    private final StrobeAnimation nick = new StrobeAnimation(240, 119, 99,
            0, 0.03, numLed);

    public LEDStripIOCANdle() {
        candle = new CANdle(candleID, Constants.CANIVORE_BUS);

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = CANdle.LEDStripType.RGB;
        config.v5Enabled = true;
        config.disableWhenLOS = true;
        config.statusLedOffWhenActive = true;
        config.vBatOutputMode = CANdle.VBatOutputMode.Off;

        candle.configAllSettings(config);
    }

    @Override
    public void ledStripOff() {
        candle.clearAnimation(0);
        candle.clearAnimation(1);
    }

    @Override
    public void idleBlueAlliance() {
        candle.animate(idleBlueAnimation,0);
        candle.animate(idleBlueAnimation, 1);
    }

    @Override
    public void idleRedAlliance() {
        candle.animate(idleRedAnimation,0);
        candle.animate(idleRedAnimation, 1);
    }

    @Override
    public void coralIntakeAnimation() {
        candle.animate(coralIntakeAnimation,0);
        candle.animate(coralIntakeAnimation, 1);
    }

    @Override
    public void nick() {
        candle.animate(nick,0);
        candle.animate(nick, 1);
    }

    @Override
    public void coralStowAnimation() {
        candle.animate(coralStowedAnimation,0);
        candle.animate(coralStowedAnimation, 1);
    }

    @Override
    public void algaeIntakeAnimation() {
        candle.animate(algaeIntakeAnimation,0);
        candle.animate(algaeIntakeAnimation, 1);
    }

    @Override
    public void progressBar() {}
}
