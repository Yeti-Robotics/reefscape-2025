package frc.robot.subsystems.led;

import frc.robot.constants.Constants;

public class ProgressBar {
    private final NewLEDSubsystem ledSubsystem;
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
                ledSubsystem.runPattern(Constants.LEDs.PROGRESS_BAR_ZERO);
                progressBarState = ProgressBarPercents.ZERO;
                break;
            case TWENTY:
                ledSubsystem.runPattern(Constants.LEDs.PROGRESS_BAR_TWENTY);
                progressBarState = ProgressBarPercents.TWENTY;
                break;
            case FORTY:
                ledSubsystem.runPattern(Constants.LEDs.PROGRESS_BAR_FORTY);
                progressBarState = ProgressBarPercents.FORTY;
                break;
            case SIXTY:
                ledSubsystem.runPattern(Constants.LEDs.PROGRESS_BAR_SIXTY);
                progressBarState = ProgressBarPercents.SIXTY;
                break;
            case EIGHTY:
                ledSubsystem.runPattern(Constants.LEDs.PROGRESS_BAR_EIGHTY);
                progressBarState = ProgressBarPercents.EIGHTY;
                break;
            case FULL:
                ledSubsystem.runPattern(Constants.LEDs.PROGRESS_BAR_FULL);
                progressBarState = ProgressBarPercents.FULL;
                break;
        }
    }

    public ProgressBar(NewLEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        progressBarState = ProgressBarPercents.ZERO;
    }
}
