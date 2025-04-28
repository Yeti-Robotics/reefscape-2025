package frc.robot.subsystems.vision.data;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record VisionRobotPose(
        Pose2d estimatedRobotPose,
        int numTags,
        Matrix<N3, N1> standardDeviations) {
    public static final Matrix<N3, N1> DEFAULT_STD_DEVS = VecBuilder.fill(1, 1, Math.toRadians(30));

    public VisionRobotPose(Pose2d estimatedRobotPose, int numTags) {
        this(estimatedRobotPose, numTags, DEFAULT_STD_DEVS);
    }
}
