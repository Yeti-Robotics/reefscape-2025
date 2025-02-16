package frc.robot.util.state;

public class StateUtils {
    public static boolean isAtSetpoint(double currentPosition, double targetPosition, double errorTolerance) {
        return Math.abs(currentPosition - targetPosition) > errorTolerance;
    }
}
