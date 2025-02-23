package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final TurnToPointRequest poseAimRequest = new TurnToPointRequest();

    private final DoubleSupplier xVelSupplier;
    private final DoubleSupplier yVelSupplier;
    private final DoubleSupplier rotationalSupplier;

    private final AprilTagSubsystem reefCam;
    private Optional<AprilTagDetection> detection;

    public ReefAlignCommand(
            CommandSwerveDrivetrain commandSwerveDrivetrain,
            AprilTagSubsystem reefCam,
            DoubleSupplier joyStickX,
            DoubleSupplier joyStickY,
            DoubleSupplier rotationalJoystick) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.reefCam = reefCam;
        this.xVelSupplier = joyStickX;
        this.yVelSupplier = joyStickY;
        this.rotationalSupplier = rotationalJoystick;

        poseAimRequest
                .withVelocityX(
                        -joyStickY.getAsDouble() * TunerConstants.kSpeedAt12Volts.magnitude())
                .withVelocityY(
                        -joyStickX.getAsDouble() * TunerConstants.kSpeedAt12Volts.magnitude());

        addRequirements(this.commandSwerveDrivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        detection = reefCam.getBestDetection();

        if (detection.isEmpty()) {
            cancel();
            return;
        }

        AprilTagDetection det = detection.get();

        Translation2d detectionTranslation = det.getTargetPose().getTranslation();

        poseAimRequest.setPointToFace(
                detectionTranslation.rotateBy(
                        new Rotation2d(0.2 * rotationalSupplier.getAsDouble())));

        commandSwerveDrivetrain.applyRequest(() -> poseAimRequest);
    }

    @Override
    public void end(boolean interrupted) {}
}
