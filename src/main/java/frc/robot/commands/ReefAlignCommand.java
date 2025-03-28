package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private static int commandCount = 0;
    private final AprilTagSubsystem reefCam1;
    private final AprilTagSubsystem reefCam2;
    private final CoralManipulatorSystem coralManipulatorSystem;

    private final SwerveRequest.FieldCentricFacingAngle swerveReq =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1);
    private final SwerveRequest.Idle stopReq = new SwerveRequest.Idle();
    AprilTagDetection lockedOnAprilTag;

    boolean isLeftBranch = false;
    boolean isFinished = false;

    PIDController movementXPIDController = new PIDController(1, 0, 0);
    PIDController movementYPIDController = new PIDController(1, 0, 0);

    private Pose2d reefTargetPose;

    public ReefAlignCommand(
            CommandSwerveDrivetrain commandSwerveDrivetrain,
            CoralManipulatorSystem coralManipulatorSystem,
            AprilTagSubsystem reefCam1,
            AprilTagSubsystem reefCam2) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.coralManipulatorSystem = coralManipulatorSystem;
        this.reefCam1 = reefCam1;
        this.reefCam2 = reefCam2;

        swerveReq.HeadingController.setPID(6, 0, 0);
        swerveReq.HeadingController.setTolerance(0.07);
        swerveReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        movementXPIDController.setTolerance(0.07);
        movementYPIDController.setTolerance(0.07);
    }

    public static StructPublisher<Pose2d> pose2dStructPublisher(String key) {
        return NetworkTableInstance.getDefault()
                .getStructTopic("ReefAlignCmd/" + key, Pose2d.struct)
                .publish();
    }

    public boolean isRedReef(int id) {
        return switch (id) {
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
                yield true;
            default:
                yield false;
        };
    }

    public boolean isBlueReef(int id) {
        return switch (id) {
            case 17:
            case 18:
            case 19:
            case 20:
            case 21:
            case 22:
                yield true;
            default:
                yield false;
        };
    }

    public Optional<AprilTagDetection> getReefCamDetection() {
        Optional<AprilTagDetection> detection1 = reefCam1.getBestDetection();
        Optional<AprilTagDetection> detection2 = reefCam2.getBestDetection();

        if (detection1.isPresent() && detection2.isPresent()) {
            int fiducial1 = detection1.get().getFiducialID();

            if (isRedReef(fiducial1) || isBlueReef(fiducial1)) {
                return detection1;
            }
        }

        return detection1.or(() -> detection2);
    }

    public Pose2d getBranchPoseFromTagID(int id) {
        DogLog.log("ReefAlignCmd/TagID", id);
        DogLog.log("ReefAlignCmd/isRedReef", isRedReef(id));
        DogLog.log("ReefAlignCmd/isBlueReef", isBlueReef(id));
        boolean isRedAllianceReef = isRedReef(id);

        if (!isRedAllianceReef && !isBlueReef(id)) {
            isFinished = true;
            return Pose2d.kZero;
        }

        int branchPoseIndex = id - (isRedAllianceReef ? 7 : 18);

        if (branchPoseIndex > 5) {
            isFinished = true;
            cancel();
            return Pose2d.kZero;
        }

        if (branchPoseIndex == -1) {
            branchPoseIndex = Reef.centerFaces.length - 1;
        }

        if (branchPoseIndex < 0) {
            isFinished = true;
            return new Pose2d();
        }

        Pose2d reefTargetPose = Reef.centerFaces[branchPoseIndex];

        return AllianceFlipUtil.apply(reefTargetPose, isRedAllianceReef);
    }

    StructPublisher<Pose2d> reefTargetPublisher = pose2dStructPublisher("ReefTarget");

    @Override
    public void initialize() {
        commandCount++;

        DogLog.log("CommandCount", commandCount);

        Optional<AprilTagDetection> detectionOpt = getReefCamDetection();

        if (detectionOpt.isEmpty()) {
            isFinished = true;
            return;
        }

        int fiducialId = detectionOpt.get().getFiducialID();

        reefTargetPose = getBranchPoseFromTagID(fiducialId);
        reefTargetPublisher.set(reefTargetPose);
    }

    @Override
    public void execute() {
        DogLog.log("ReefAlignCmd/TargetPoseNull", reefTargetPose == null);
        if (reefTargetPose == null) {
            isFinished = true;
            return;
        }

        Pose2d drivetrainPose = commandSwerveDrivetrain.getState().Pose;

        double veloX =
                movementXPIDController.calculate(drivetrainPose.getX(), reefTargetPose.getX());

        double veloY =
                movementYPIDController.calculate(drivetrainPose.getY(), reefTargetPose.getY());

        DogLog.log("ReefAlignCmd/XVelocity", veloX);
        DogLog.log("ReefAlignCmd/YVelocity", veloY);

        veloX = MathUtil.clamp(veloX, -3, 3);
        veloY = MathUtil.clamp(veloY, -3, 3);

        commandSwerveDrivetrain.setControl(
                swerveReq
                        .withVelocityX(-veloX)
                        .withVelocityY(-veloY)
                        .withTargetDirection(reefTargetPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        commandSwerveDrivetrain.setControl(stopReq);
        reefTargetPose = null;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public Command toggleBranchSelection() {
        return Commands.runOnce(() -> isLeftBranch = !isLeftBranch);
    }
}
