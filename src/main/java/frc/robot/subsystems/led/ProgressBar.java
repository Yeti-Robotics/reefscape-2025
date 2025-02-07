package frc.robot.subsystems.led;

public class ProgressBar {
    private LEDSubsystem ledSubsystem;
    public static ProgressBarPercents progressBarState;

    public enum ProgressBarPercents {
        ZERO,
        TWENTY,
        FORTY,
        SIXTY,
        EIGHTY,
        FULL;
    }

    public void addProgress() {
        if (progressBarState == ProgressBarPercents.TWENTY) {
            progressBarState = ProgressBarPercents.FORTY;
        } else if (progressBarState == ProgressBarPercents.FORTY) {
            progressBarState = ProgressBarPercents.SIXTY;
        } else if (progressBarState == ProgressBarPercents.SIXTY) {
            progressBarState = ProgressBarPercents.EIGHTY;
        } else if (progressBarState == ProgressBarPercents.EIGHTY) {
            progressBarState = ProgressBarPercents.FULL;
        }
    }

    public void subtractProgress() {
        if (progressBarState == ProgressBarPercents.FULL) {
            progressBarState = ProgressBarPercents.EIGHTY;
        } else if (progressBarState == ProgressBarPercents.EIGHTY) {
            progressBarState = ProgressBarPercents.SIXTY;
        } else if (progressBarState == ProgressBarPercents.SIXTY) {
            progressBarState = ProgressBarPercents.FORTY;
        } else if (progressBarState == ProgressBarPercents.FORTY) {
            progressBarState = ProgressBarPercents.TWENTY;
        }
    }

    public void setProgress(ProgressBarPercents type) {
        switch (type) {
            case ZERO:
                ledSubsystem.candle.setLEDs(255, 0, 0);
                progressBarState = ProgressBarPercents.ZERO;
                break;
            case TWENTY:
                ledSubsystem.candle.setLEDs(0, 0, 255, 0, 0, 11);
                ledSubsystem.candle.setLEDs(255, 0, 0, 0, 11, 24);
                progressBarState = ProgressBarPercents.TWENTY;
                break;
            case FORTY:
                ledSubsystem.candle.setLEDs(0, 0, 255, 0, 0, 13);
                ledSubsystem.candle.setLEDs(255, 0, 0, 0, 13, 24);
                progressBarState = ProgressBarPercents.FORTY;
                break;
            case SIXTY:
                ledSubsystem.candle.setLEDs(0, 0, 255, 0, 0, 16);
                ledSubsystem.candle.setLEDs(255, 0, 0, 0, 16, 24);
                progressBarState = ProgressBarPercents.SIXTY;
                break;
            case EIGHTY:
                ledSubsystem.candle.setLEDs(0, 0, 255, 0, 0, 19);
                ledSubsystem.candle.setLEDs(255, 0, 0, 0, 19, 24);
                progressBarState = ProgressBarPercents.EIGHTY;
                break;
            case FULL:
                ledSubsystem.candle.setLEDs(0, 0, 255);
                progressBarState = ProgressBarPercents.FULL;
                break;
        }
    }

    public ProgressBar(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
    }
}
