package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private final AprilTagSubsystem reefCam1;
    private final AprilTagSubsystem reefCam2;
    private Pose2d currPose;
    private AprilTagDetection detection;
    private Pose2d tagPose;
    private final Transform2d branchLeftPose =
            new Transform2d(
                    Units.inchesToMeters(-8.04), Units.inchesToMeters(6.47), new Rotation2d());
    private final Transform2d branchRightPose =
            new Transform2d(
                    Units.inchesToMeters(8.04), Units.inchesToMeters(6.47), new Rotation2d());
    private final SwerveRequest.RobotCentricFacingAngle swerveReq =
            new SwerveRequest.RobotCentricFacingAngle();

    public ReefAlignCommand(
            CommandSwerveDrivetrain commandSwerveDrivetrain,
            AprilTagSubsystem reefCam1,
            AprilTagSubsystem reefCam2) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.reefCam1 = reefCam1;
        this.reefCam2 = reefCam2;

        swerveReq.HeadingController.setPID(8.2032, 0, 0.97656);
        swerveReq.HeadingController.setTolerance(0.07);
        swerveReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}
}
