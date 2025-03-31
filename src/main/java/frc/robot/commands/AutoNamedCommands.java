package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.coral.*;
import frc.robot.subsystems.coral.grabber.GrabberState;
import java.util.Set;

public class AutoNamedCommands {
    private final CoralManipulatorSystem coralManipulator;
    private final ReefAlignPPOTF reefAlignCommand;

    public AutoNamedCommands(
            CoralManipulatorSystem coralManipulator, ReefAlignPPOTF reefAlignCommand) {
        this.coralManipulator = coralManipulator;
        this.reefAlignCommand = reefAlignCommand;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand(
                "GroundIntake", coralManipulator.transitionTo(CoralManipulatorState.GROUND_INTAKE));
        NamedCommands.registerCommand(
                "HPIntake", coralManipulator.transitionTo(CoralManipulatorState.HP_INTAKE));
        NamedCommands.registerCommand(
                "LollipopIntake", coralManipulator.transitionTo(CoralManipulatorState.LOLLIPOP));

        NamedCommands.registerCommand(
                "ReefAlign", Commands.defer(reefAlignCommand::autoAlign, Set.of()));

        NamedCommands.registerCommand(
                "L1", coralManipulator.transitionTo(CoralManipulatorState.L1));
        NamedCommands.registerCommand(
                "L2", coralManipulator.transitionTo(CoralManipulatorState.L2));
        NamedCommands.registerCommand(
                "L3", coralManipulator.transitionTo(CoralManipulatorState.L3));
        NamedCommands.registerCommand(
                "L4", coralManipulator.transitionTo(CoralManipulatorState.L4));

        NamedCommands.registerCommand(
                "Rollout", coralManipulator.grabber.transitionTo(GrabberState.ROLL_OUT));

        NamedCommands.registerCommand(
                "Score1", coralManipulator.transitionTo(CoralManipulatorState.SCORE_L1));
        NamedCommands.registerCommand(
                "Score2", coralManipulator.transitionTo(CoralManipulatorState.SCORE_L2));
        NamedCommands.registerCommand(
                "Score3", coralManipulator.transitionTo(CoralManipulatorState.SCORE_L3));
        NamedCommands.registerCommand(
                "Score4", coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4));

        NamedCommands.registerCommand(
                "AlignAndScoreL4",
                new SequentialCommandGroup(
                    Commands.defer(reefAlignCommand::autoAlign, Set.of()),
                        coralManipulator.transitionTo(CoralManipulatorState.L4),
                        coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4),
                        new WaitCommand(1),
                        coralManipulator.transitionTo(CoralManipulatorState.STOWED)
                )
        );
        NamedCommands.registerCommand(
                "Stow", coralManipulator.transitionTo(CoralManipulatorState.STOWED));
    }
}
