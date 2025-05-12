package frc.robot.subsystems.vision.io.impl.limelight.util;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Notifier;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class LimelightDataParsingHelper {
    private static final LimelightHelpers.LimelightResults DEFAULT_RESULTS = new LimelightHelpers.LimelightResults();
    private static Frequency UPDATE_FREQUENCY = Units.Hertz.of(50.0);

    private static final Map<String, LimelightHelpers.LimelightResults> limelightResultsMap = new ConcurrentHashMap<>();
    private static LimelightDataParsingHelper instance;
    private final Notifier notifier;
    private boolean started = false;

    private LimelightDataParsingHelper() {
        notifier = new Notifier(this::updateLimelightData);
    }

    public static synchronized LimelightDataParsingHelper getInstance() {
        if (instance == null) {
            instance = new LimelightDataParsingHelper();
        }
        return instance;
    }

    public static void setUpdateFrequency(Frequency updateFrequency) {
        UPDATE_FREQUENCY = updateFrequency;
    }

    public void start() {
        if (!started) {
            notifier.startPeriodic(1 / UPDATE_FREQUENCY.in(Units.Hertz));
            started = true;
        }
    }

    public void stop() {
        notifier.stop();
        started = false;
    }

    private void updateLimelightData() {
        for (String limelightName : limelightResultsMap.keySet()) {
            LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

            if (results.valid) {
                limelightResultsMap.put(limelightName, results);
            }
        }
    }

    private static LimelightHelpers.LimelightResults createParserFor(String limelightName) {
        limelightResultsMap.putIfAbsent(limelightName, DEFAULT_RESULTS);

        if (!getInstance().started) {
            getInstance().start();
        }

        return DEFAULT_RESULTS;
    }

    public static void removeParserFor(String limelightName) {
        limelightResultsMap.remove(limelightName);

        if (limelightResultsMap.isEmpty()) {
            getInstance().stop();
        }
    }

    /**
     *
     * @param limelightName name of the limelight to get results for
     * @return limelight json results
     */
    public static LimelightHelpers.LimelightResults getResults(String limelightName) {
        return limelightResultsMap.computeIfAbsent(limelightName, LimelightDataParsingHelper::createParserFor);
    }
}
