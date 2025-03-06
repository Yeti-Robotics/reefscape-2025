package frc.robot.subsystems.led;

public class ProgressBar {
    private final LEDSubsystem ledSubsystem;
    public ProgressBarPercents progressBarState;

    public enum ProgressBarPercents {
        ZERO(0),
        TWENTY(20),
        FORTY(40),
        SIXTY(60),
        EIGHTY(80),
        FULL(100);

        private final int progress;

        ProgressBarPercents(int progress) {
            this.progress = progress;
        }

        public int getProgress() {
            return progress;
        }
    }

    public void addProgress() {
        switch (progressBarState) {
            case ZERO -> progressBarState = ProgressBarPercents.TWENTY;
            case TWENTY -> progressBarState = ProgressBarPercents.FORTY;
            case FORTY -> progressBarState = ProgressBarPercents.SIXTY;
            case SIXTY -> progressBarState = ProgressBarPercents.EIGHTY;
            case EIGHTY -> progressBarState = ProgressBarPercents.FULL;
        }
    }

    public void subtractProgress() {
        switch (progressBarState) {
            case FULL -> progressBarState = ProgressBarPercents.EIGHTY;
            case EIGHTY -> progressBarState = ProgressBarPercents.SIXTY;
            case SIXTY -> progressBarState = ProgressBarPercents.FORTY;
            case FORTY -> progressBarState = ProgressBarPercents.TWENTY;
            case TWENTY -> progressBarState = ProgressBarPercents.ZERO;
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
        progressBarState = ProgressBarPercents.ZERO;
    }
}
