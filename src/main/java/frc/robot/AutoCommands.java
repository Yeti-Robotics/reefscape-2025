package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ReefAlignPPOTF;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.PathPlannerUtils;
import java.util.Optional;

public class AutoCommands {
    private final CoralManipulatorSystem coralManipulator;
    private final ReefAlignPPOTF reefAlignPPOTF;
    private final CommandSwerveDrivetrain drivetrain;

    public AutoCommands(
            CoralManipulatorSystem coralManipulator,
            ReefAlignPPOTF reefAlignPPOTF,
            CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.coralManipulator = coralManipulator;
        this.reefAlignPPOTF = reefAlignPPOTF;
    }

    public Command right1Pc() {
        Optional<PathPlannerPath> lineF = PathPlannerUtils.loadPathByName("lineF");

        return lineF.isEmpty()
                ? Commands.none()
                : AutoBuilder.followPath(lineF.get())
                        .andThen(
                                reefAlignPPOTF.setBranch(ReefAlignPPOTF.Branch.LEFT).withTimeout(1))
                        .andThen(reefAlignPPOTF.reefAlign())
                        .andThen(
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.L4)
                                        .withTimeout(1))
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4))
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.STOWED));
    }

    public Command left2PcLolli() {
        Optional<PathPlannerPath> lineJ = PathPlannerUtils.loadPathByName("lineJ");
        Optional<PathPlannerPath> jToLollipop = PathPlannerUtils.loadPathByName("jToLoli");
        Optional<PathPlannerPath> lollipopToL = PathPlannerUtils.loadPathByName("loliToL");

        PathPlannerAuto auto;
        PathPlannerAuto lollipathJ =
                new PathPlannerAuto((AutoBuilder.followPath(jToLollipop.get())));
        PathPlannerAuto lollipathL =
                new PathPlannerAuto((AutoBuilder.followPath(lollipopToL.get())));

        var cmd =
                lineJ.isEmpty() || jToLollipop.isEmpty()
                        ? Commands.none()
                        : Commands.sequence(
                                AutoBuilder.followPath(lineJ.get()),
                                reefAlignPPOTF.setBranch(ReefAlignPPOTF.Branch.LEFT),
                                reefAlignPPOTF.reefAlign(),
                                coralManipulator.transitionTo(CoralManipulatorState.L4),
                                coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4),
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(0.5),
                                AutoBuilder.followPath(jToLollipop.get())
                                        .until(coralManipulator.grabber::hasCoral),
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(1),
                                AutoBuilder.followPath(lollipopToL.get()),
                                reefAlignPPOTF.reefAlign(),
                                coralManipulator.transitionTo(CoralManipulatorState.CLIMB_L4),
                                coralManipulator.transitionTo(CoralManipulatorState.SCORE_CLIMB_L4),
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(0.5));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command right2PcLolli() {
        Optional<PathPlannerPath> lineF = PathPlannerUtils.loadPathByName("lineF");
        Optional<PathPlannerPath> fToLollipop = PathPlannerUtils.loadPathByName("fToLoli");
        Optional<PathPlannerPath> lollipopToD = PathPlannerUtils.loadPathByName("loliToD");

        PathPlannerAuto auto;

        var cmd =
                lineF.isEmpty() || fToLollipop.isEmpty() || lollipopToD.isEmpty()
                        ? Commands.none()
                        : Commands.sequence(
                                AutoBuilder.followPath(lineF.get()),
                                reefAlignPPOTF.setBranch(ReefAlignPPOTF.Branch.LEFT),
                                reefAlignPPOTF.reefAlign(),
                                coralManipulator.transitionTo(CoralManipulatorState.L4),
                                coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4),
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(0.5)
                                        .andThen(
                                                Commands.sequence(
                                                                AutoBuilder.followPath(
                                                                        fToLollipop.get()),
                                                                coralManipulator
                                                                        .transitionTo(
                                                                                CoralManipulatorState
                                                                                        .STOWED)
                                                                        .withTimeout(0.5),
                                                                AutoBuilder.followPath(
                                                                        lollipopToD.get()),
                                                                reefAlignPPOTF.reefAlign(),
                                                                coralManipulator.transitionTo(
                                                                        CoralManipulatorState
                                                                                .CLIMB_L4),
                                                                coralManipulator.transitionTo(
                                                                        CoralManipulatorState
                                                                                .SCORE_CLIMB_L4),
                                                                coralManipulator
                                                                        .transitionTo(
                                                                                CoralManipulatorState
                                                                                        .STOWED)
                                                                        .withTimeout(0.5))
                                                        .onlyIf(
                                                                coralManipulator.grabber
                                                                        ::doesNotHaveCoral)));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command skipLeft() {
        Optional<PathPlannerPath> skipLeft = PathPlannerUtils.loadPathByName("skipLeft");
        Optional<PathPlannerPath> AtoLolli2 = PathPlannerUtils.loadPathByName("AtoLolli2");

        return skipLeft.isEmpty() || AtoLolli2.isEmpty()
                ? Commands.none()
                : AutoBuilder.followPath(skipLeft.get())
                        .andThen(reefAlignPPOTF.setBranch(ReefAlignPPOTF.Branch.RIGHT))
                        .andThen(reefAlignPPOTF.reefAlign())
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.L4))
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4))
                        .andThen(
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(1));
    }

    public Command left1Pc() {
        Optional<PathPlannerPath> lineJ = PathPlannerUtils.loadPathByName("lineJ");

        return lineJ.isEmpty()
                ? Commands.none()
                : AutoBuilder.followPath(lineJ.get())
                        .andThen(reefAlignPPOTF.setBranch(ReefAlignPPOTF.Branch.RIGHT))
                        .andThen(reefAlignPPOTF.reefAlign())
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.L4))
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4))
                        .andThen(
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(1));
    }

    public Command driveForward() {
        return drivetrain.applyRequest(
                () -> new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(1, 0, 0)));
    }

    public Command mid1Pc() {
        Optional<PathPlannerPath> lineG = PathPlannerUtils.loadPathByName("lineG");
        return lineG.isEmpty()
                ? Commands.none()
                : AutoBuilder.followPath(lineG.get())
                        .andThen(reefAlignPPOTF.setBranch(ReefAlignPPOTF.Branch.LEFT))
                        .andThen(reefAlignPPOTF.reefAlign())
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.L4))
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4))
                        .andThen(
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(0.5));
    }

    public SendableChooser<Command> buildAutoCommandChooser() {
        SendableChooser<Command> auto = new SendableChooser<>();
        auto.addOption("Right 1 Piece", right1Pc());
        auto.addOption("Left 1 Piece", left1Pc());
        auto.addOption("Right 2 Piece Lollipop", right2PcLolli());
        auto.addOption("Left 2 Piece Lollipop", left2PcLolli());
        auto.addOption("Mid 1 Piece", mid1Pc());
        auto.addOption("Drive forward", driveForward());
        auto.addOption("Skip Left", skipLeft());

        auto.setDefaultOption("Drive forward", driveForward());
        return auto;
    }
}
