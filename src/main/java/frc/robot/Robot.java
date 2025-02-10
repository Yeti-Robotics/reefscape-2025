// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.AprilTagCamSimBuilder;
import frc.robot.util.AprilTagSimulator;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;
    AprilTagSimulator tagSimulator;

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        tagSimulator = new AprilTagSimulator();

        tagSimulator.addCamera(
                AprilTagCamSimBuilder.newCamera()
                        .withCameraName("elevCam1")
                        .withTranslation(
                                Units.inchesToMeters(-8),
                                Units.inchesToMeters(-7),
                                Units.inchesToMeters(22.5))
                        .withRotation(0, Math.toRadians(35), Math.toRadians(90))
                        .build());
        tagSimulator.addCamera(
                AprilTagCamSimBuilder.newCamera()
                        .withCameraName("elevCam2")
                        .withTranslation(
                                Units.inchesToMeters(-8),
                                Units.inchesToMeters(7),
                                Units.inchesToMeters(13.5 + 19))
                        .withRotation(0, Math.toRadians(35), Math.toRadians(-90))
                        .build());
        tagSimulator.addCamera(
                AprilTagCamSimBuilder.newCamera()
                        .withCameraName("rearCam")
                        .withTranslation(
                                Units.inchesToMeters(-9.5),
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(35.125))
                        .withRotation(0, 0, Math.toRadians(-180))
                        .build());
    }

    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        tagSimulator.update(robotContainer.drivetrain.getState().Pose);
    }

    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
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
    public void simulationPeriodic() {}
}
