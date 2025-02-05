package frc.robot.subsystems.led;

public class ProgressBar {
    private LEDSubsystem ledSubsystem;

    public enum ProgressBarPercents {
        ZERO,
        TWENTY,
        FORTY,
        SIXTY,
        EIGHTY,
        FULL;
    }

    public void setProgress(ProgressBarPercents type) {
        switch (type) {
            case ZERO:
                ledSubsystem.candle.setLEDs(255, 0, 0);
                break;
            case TWENTY:
                ledSubsystem.candle.setLEDs(0, 0, 255, 0, 0, 11);
                ledSubsystem.candle.setLEDs(255, 0, 0, 0, 11, 24);
                break;
            case FORTY:
                ledSubsystem.candle.setLEDs(0, 0, 255, 0, 0, 13);
                ledSubsystem.candle.setLEDs(255, 0, 0, 0, 13, 24);
                break;
            case SIXTY:
                ledSubsystem.candle.setLEDs(0, 0, 255, 0, 0, 16);
                ledSubsystem.candle.setLEDs(255, 0, 0, 0, 16, 24);
                break;
            case EIGHTY:
                ledSubsystem.candle.setLEDs(0, 0, 255, 0, 0, 19);
                ledSubsystem.candle.setLEDs(255, 0, 0, 0, 19, 24);
                break;
            case FULL:
                ledSubsystem.candle.setLEDs(0, 0, 255);
                break;
        }
    }

    public ProgressBar(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
    }
}
