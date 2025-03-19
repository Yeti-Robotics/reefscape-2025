package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import java.util.Optional;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private final AprilTagSubsystem reefCam1;
    private final AprilTagSubsystem reefCam2;
    private Pose2d currPose;
    private AprilTagDetection detection;
    private Pose2d tagPose;
    private final Transform2d branchLeftPose =
            new Transform2d(
                    edu.wpi.first.units.Units.Meters.of(Units.inchesToMeters(8.04)),
                    edu.wpi.first.units.Units.Meters.of(Units.inchesToMeters(6.47)),
                    Rotation2d.fromDegrees(45));
    private final Transform2d branchRightPose =
            new Transform2d(
                    edu.wpi.first.units.Units.Meters.of(Units.inchesToMeters(-8.04)),
                    edu.wpi.first.units.Units.Meters.of(Units.inchesToMeters(6.47)),
                    Rotation2d.fromDegrees(-45));
    private final SwerveRequest.RobotCentricFacingAngle swerveReq =
            new SwerveRequest.RobotCentricFacingAngle();
    private final SwerveRequest.SwerveDriveBrake stopReq = new SwerveRequest.SwerveDriveBrake();
    AprilTagDetection lockedOnAprilTag;
    boolean isFinished = false;
    boolean isLeftBranch = false;
    ProfiledPIDController movementPIDController =
            new ProfiledPIDController(
                    0,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(
                            TunerConstants.MAX_VELOCITY_METERS_PER_SECOND - 1, 3.0));
    Pose2d initialDrivePose;

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

    public Optional<AprilTagDetection> getReefCamDetection() {
        return reefCam1.getBestDetection().or(reefCam2::getBestDetection);
    }

    @Override
    public void initialize() {
        isFinished = false;
        Optional<AprilTagDetection> reefCamDetection = getReefCamDetection();
        if (reefCamDetection.isEmpty()) {
            isFinished = true;
            return;
        }

        initialDrivePose = commandSwerveDrivetrain.getState().Pose;
        lockedOnAprilTag = reefCamDetection.get();
    }

    Field2d field = new Field2d();

    @Override
    public void execute() {
        Optional<AprilTagDetection> reefCamDetectionOpt = getReefCamDetection();
        if (reefCamDetectionOpt.isEmpty()) {
            isFinished = true;
            return;
        }

        AprilTagDetection reefCamDetection = reefCamDetectionOpt.get();

        if ((lockedOnAprilTag == null)
                || reefCamDetection.getFiducialID() != lockedOnAprilTag.getFiducialID()) {
            isFinished = true;
            return;
        }

        Pose2d targetVisionPose = reefCamDetection.getRobotToTargetPose();
        Pose2d targetBranchPose =
                targetVisionPose.transformBy(isLeftBranch ? branchLeftPose : branchRightPose);
        field.setRobotPose(
                reefCamDetection
                        .getRobotInFieldPose()
                        .transformBy(
                                new Transform2d(
                                        targetBranchPose.getTranslation(),
                                        targetVisionPose.getRotation())));
        SmartDashboard.putData("ATarget Branch Pose", field);

        commandSwerveDrivetrain.setControl(
                swerveReq
                        .withTargetDirection(
                                targetBranchPose.getRotation().plus(Rotation2d.fromDegrees(90)))
                        .withVelocityX(targetBranchPose.getX())
                        .withVelocityY(targetBranchPose.getY()));
    }

    @Override
    public void end(boolean interrupted) {
        commandSwerveDrivetrain.setControl(stopReq);
    }

    public Command toggleBranchSelection() {
        return Commands.runOnce(() -> isLeftBranch = !isLeftBranch);
    }
}
