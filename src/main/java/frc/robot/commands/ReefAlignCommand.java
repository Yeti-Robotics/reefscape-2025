package frc.robot.commands;

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

    private final TurnToPointRequest poseAimReq = new TurnToPointRequest();
    private final DoubleSupplier xVelSupplier;
    private final DoubleSupplier yVelSupplier;

    private final AprilTagSubsystem reefCam;
    private AprilTagDetection detection;

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
        Optional<AprilTagDetection> aprilTagDetectionOpt = reefCam.getBestDetection();

        if (aprilTagDetectionOpt.isEmpty()) {
            cancel();
            return;
        }

        detection = aprilTagDetectionOpt.get();
        poseAimReq.setTargetPoseRelativeToRobot(detection.getTargetPose());
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber(
                "Robot/robotContainer/Vision/Limelight/Turn error",
                poseAimReq.HeadingController.getPositionError());

        Optional<AprilTagDetection> aprilTagDetectionOptional = reefCam.getBestDetection();

        if (aprilTagDetectionOptional.isEmpty()
                || aprilTagDetectionOptional.get().getFiducialID() != detection.getFiducialID()) {
            end(false);
            return;
        }

        AprilTagDetection robotDetection = aprilTagDetectionOptional.get();

        commandSwerveDrivetrain.setControl(
                poseAimReq
                        .withVelocityX(
                                -xVelSupplier.getAsDouble()
                                        * TunerConstants.kSpeedAt12Volts.magnitude())
                        .withVelocityY(
                                -yVelSupplier.getAsDouble()
                                        * TunerConstants.kSpeedAt12Volts.magnitude()));
    }

    @Override
    public void end(boolean interrupted) {}
}
