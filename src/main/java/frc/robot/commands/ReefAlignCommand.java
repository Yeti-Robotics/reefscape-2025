package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private final SwerveRequest.FieldCentricFacingAngle poseAimReq;
    private final DoubleSupplier xVelSupplier;
    private final DoubleSupplier yVelSupplier;

    private final AprilTagSubsystem reefCam;
    private Pose2d currPose;
    private AprilTagDetection detection;
    private Pose2d tagPose;

    public ReefAlignCommand(
            CommandSwerveDrivetrain commandSwerveDrivetrain,
            AprilTagSubsystem reefCam,
            DoubleSupplier joyStickX,
            DoubleSupplier joyStickY) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.reefCam = reefCam;
        this.xVelSupplier = joyStickX;
        this.yVelSupplier = joyStickY;

        addRequirements(this.commandSwerveDrivetrain);
        poseAimReq = new SwerveRequest.FieldCentricFacingAngle();
        poseAimReq.HeadingController.setPID(5, 0, 0);
        poseAimReq.HeadingController.setTolerance(0.07);
        poseAimReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        System.out.println("Reef cmd init");
        this.currPose = commandSwerveDrivetrain.getState().Pose;
        Optional<AprilTagDetection> aprilTagDetectionOpt = reefCam.getBestDetection();
        SmartDashboard.putBoolean("ATag present", aprilTagDetectionOpt.isPresent());
        if (aprilTagDetectionOpt.isEmpty()) {
            cancel();
            return;
        }

        detection = aprilTagDetectionOpt.get();
        tagPose = detection.getTargetPose();
    }

    Field2d field = new Field2d();

    @Override
    public void execute() {
        System.out.println("tag id: " + detection.getFiducialID());
        field.setRobotPose(tagPose);
        SmartDashboard.putData("ADetection Pose", field);

        // Apply drive control with joystick inputs
        commandSwerveDrivetrain.setControl(
                poseAimReq
                        .withTargetDirection(
                                tagPose.getTranslation()
                                        .minus(currPose.getTranslation())
                                        .getAngle())
                        .withVelocityX(
                                -xVelSupplier.getAsDouble()
                                        * TunerConstants.kSpeedAt12Volts.magnitude())
                        .withVelocityY(
                                -yVelSupplier.getAsDouble()
                                        * TunerConstants.kSpeedAt12Volts.magnitude()));
        SmartDashboard.putNumber(
                "ADetection Error", poseAimReq.HeadingController.getPositionError());
    }
}
