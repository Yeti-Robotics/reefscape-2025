package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    private final SwerveRequest.RobotCentricFacingAngle poseAimReq =
            new SwerveRequest.RobotCentricFacingAngle();
    private final DoubleSupplier xVelSupplier;
    private final DoubleSupplier yVelSupplier;

    private final AprilTagSubsystem reefCam;
    private AprilTagDetection previousDetection;

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

        poseAimReq.HeadingController.setPID(5, 0, 0);
        poseAimReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        System.out.println("Reef cmd init");
    }

    Field2d field = new Field2d();

    @Override
    public void execute() {

        Optional<AprilTagDetection> aprilTagDetectionOptional = reefCam.getBestDetection();

        if (aprilTagDetectionOptional.isEmpty()) {
            return;
        }

        AprilTagDetection detection = aprilTagDetectionOptional.get();

        if (previousDetection == null) {
            previousDetection = detection;
        } else if (previousDetection.getFiducialID() != detection.getFiducialID()) {
            return;
        }

        Pose2d tagPose = detection.getTargetPose();

        System.out.println("tag id: " + detection.getFiducialID());
        Pose2d robotPose = commandSwerveDrivetrain.getState().Pose;
        field.setRobotPose(
                robotPose.transformBy(
                        new Transform2d(tagPose.getTranslation(), tagPose.getRotation())));
        SmartDashboard.putData("ADetection Pose", field);

        // Apply drive control with joystick inputs
        commandSwerveDrivetrain.setControl(
                poseAimReq
                        .withTargetDirection(
                                commandSwerveDrivetrain
                                        .getPigeon2()
                                        .getRotation2d()
                                        .rotateBy(tagPose.getRotation())
                                        .rotateBy(new Rotation2d(Math.PI / 2)))
                        .withVelocityX(
                                -xVelSupplier.getAsDouble()
                                        * TunerConstants.kSpeedAt12Volts.magnitude())
                        .withVelocityY(
                                -yVelSupplier.getAsDouble()
                                        * TunerConstants.kSpeedAt12Volts.magnitude()));
    }

    @Override
    public void end(boolean interrupted) {
        previousDetection = null;
    }
}
