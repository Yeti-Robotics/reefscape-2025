// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.util.Elastic;
import frc.robot.util.sim.PhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Logged
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        Epilogue.bind(this);
        SmartDashboard.putString("Build Date", BuildConstants.BUILD_DATE);
        SmartDashboard.putString("GIT SHA", BuildConstants.GIT_SHA);
        SmartDashboard.putString("Git Branch", BuildConstants.GIT_BRANCH);
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    String llName = "limelight";

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.updateMechanisms();
        robotContainer.updateVision();
    }

    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        Elastic.selectTab("Prematch");
    }

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        Elastic.selectTab("Autonomous");

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        Elastic.selectTab("Teleoperated");
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        robotContainer
                .coralManipulator
                .transitionTo(CoralManipulatorState.IDLE)
                .ignoringDisable(true)
                .schedule();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This method is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This method is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        robotContainer.updateVisionSim();
        PhysicsSim.getInstance().run();
    }
}
