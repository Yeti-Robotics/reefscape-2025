package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class DriveForwardALittleCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private static final ProfiledPIDController movementPIDController =
            new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(3.5, 3.0));
    private final SwerveRequest.RobotCentricFacingAngle swerveReq =
            new SwerveRequest.RobotCentricFacingAngle()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1);
    private final SwerveRequest.Idle stopReq = new SwerveRequest.Idle();
    private Rotation2d currentHeading;
    private Pose2d startingPose;
    private Distance distanceToMove;
    private boolean finished = false;

    public DriveForwardALittleCommand(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.commandSwerveDrivetrain);
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        startingPose = commandSwerveDrivetrain.getState().Pose;
        currentHeading = startingPose.getRotation();
        if (distanceToMove == null) {
            finished = true;
        }
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
     * called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (currentHeading == null || startingPose == null) {
            finished = true;
            return;
        }
        Transform2d transformToCurrent =
                new Transform2d(startingPose, commandSwerveDrivetrain.getState().Pose);
        double movementOutput =
                movementPIDController.calculate(
                        transformToCurrent.getTranslation().getNorm(),
                        distanceToMove.in(Units.Meters));
        double normalizeMovementOutput =
                Math.copySign(Math.sqrt(Math.abs(movementOutput)), movementOutput);
        commandSwerveDrivetrain.setControl(
                swerveReq
                        .withVelocityX(
                                normalizeMovementOutput * Math.cos(currentHeading.getRadians()))
                        .withVelocityY(
                                normalizeMovementOutput * Math.sin(currentHeading.getRadians()))
                        .withTargetDirection(currentHeading));
    }

    /**
     * Returns whether this command has finished. Once a command finishes -- indicated by this
     * method returning true -- the scheduler will call its {@link #end(boolean)} method.
     *
     * <p>Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for
     * such an operation.
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally --
     * that is it is called when {@link #isFinished()} returns true -- or when it is
     * interrupted/canceled. This is where you may want to wrap up loose ends, like shutting off a
     * motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        commandSwerveDrivetrain.setControl(stopReq);
        currentHeading = null;
        startingPose = null;
    }

    public DriveForwardALittleCommand withDistance(Distance distance) {
        distanceToMove = distance;
        return this;
    }
}
