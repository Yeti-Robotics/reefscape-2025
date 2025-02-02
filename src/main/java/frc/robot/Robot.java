// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.*;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;
    VisionSystemSim visionSim;
    StructArrayPublisher<Pose3d> elevCam1TagPub;
    StructArrayPublisher<Pose3d> elevCam2TagPub;
    StructArrayPublisher<Pose3d> rearCamTagPub;
    StructArrayPublisher<Pose3d> tagPublisher;
    PhotonCamera elevCam1;
    PhotonCamera elevCam2;
    PhotonCamera rearCam;
    List<Integer> elevCam1Cache;
    List<Integer> elevCam2Cache;
    List<Integer> rearCamCache;

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        visionSim = new VisionSystemSim("main");
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        visionSim.addAprilTags(tagLayout);
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        // The PhotonCamera used in the real robot code.
        elevCam1 = new PhotonCamera("elevCam1");
        elevCam2 = new PhotonCamera("elevCam2");
        rearCam = new PhotonCamera("rearCam");

        // The simulation of this camera. Its values used in real robot code will be updated.
        PhotonCameraSim elevCam1Sim = new PhotonCameraSim(elevCam1, cameraProp);
        PhotonCameraSim elevCam2Sim = new PhotonCameraSim(elevCam2, cameraProp);
        PhotonCameraSim rearCamSim = new PhotonCameraSim(rearCam, cameraProp);
        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToElevCam1Trl =
                new Translation3d(
                        Units.inchesToMeters(-8),
                        Units.inchesToMeters(-7),
                        Units.inchesToMeters(22.5));
        Translation3d robotToElevCam2Trl =
                new Translation3d(
                        Units.inchesToMeters(-8),
                        Units.inchesToMeters(7),
                        Units.inchesToMeters(22.5));
        Translation3d robotToRearCamTrl =
                new Translation3d(Units.inchesToMeters(-9.5), 0, Units.inchesToMeters(35.125));
        // and pitched 15 degrees up.
        Rotation3d robotToElevCam1Rot = new Rotation3d(0, Math.toRadians(35), Math.toRadians(90));
        Rotation3d robotToElevCam2Rot = new Rotation3d(0, Math.toRadians(35), Math.toRadians(-90));
        Rotation3d robotToRearCamRot = new Rotation3d(0, 0, Math.toRadians(180));

        Transform3d robotToElevCam1 = new Transform3d(robotToElevCam1Trl, robotToElevCam1Rot);
        Transform3d robotToElevCam2 = new Transform3d(robotToElevCam2Trl, robotToElevCam2Rot);
        Transform3d robotToRearCam = new Transform3d(robotToRearCamTrl, robotToRearCamRot);
        // Enable the raw and processed streams. These are enabled by default.
        elevCam1Sim.enableRawStream(true);
        elevCam1Sim.enableProcessedStream(true);
        elevCam2Sim.enableRawStream(true);
        elevCam2Sim.enableProcessedStream(true);
        rearCamSim.enableRawStream(true);
        rearCamSim.enableProcessedStream(true);

        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        elevCam1Sim.enableDrawWireframe(true);
        elevCam2Sim.enableDrawWireframe(true);
        rearCamSim.enableDrawWireframe(true);

        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        visionSim.addCamera(elevCam1Sim, robotToElevCam1);
        visionSim.addCamera(elevCam2Sim, robotToElevCam2);
        visionSim.addCamera(rearCamSim, robotToRearCam);
        elevCam1TagPub =
                NetworkTableInstance.getDefault()
                        .getStructArrayTopic("elevCam1SeenTags", Pose3d.struct)
                        .publish();
        elevCam2TagPub =
                NetworkTableInstance.getDefault()
                        .getStructArrayTopic("elevCam2SeenTags", Pose3d.struct)
                        .publish();
        rearCamTagPub =
                NetworkTableInstance.getDefault()
                        .getStructArrayTopic("rearCamSeenTags", Pose3d.struct)
                        .publish();
        tagPublisher =
                NetworkTableInstance.getDefault()
                        .getStructArrayTopic("allTags", Pose3d.struct)
                        .publish();
        elevCam1Cache = new ArrayList<>();
        elevCam2Cache = new ArrayList<>();
        rearCamCache = new ArrayList<>();
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
        visionSim.update(robotContainer.drivetrain.getState().Pose);
        Pose3d[] ts =
                visionSim.getVisionTargets().stream()
                        .map(VisionTargetSim::getPose)
                        .toArray(Pose3d[]::new);
        tagPublisher.set(ts);

        elevCam1TagPub.set(getSeenTags(getCamTargets(elevCam1, elevCam1Cache)));
        elevCam2TagPub.set(getSeenTags(getCamTargets(elevCam2, elevCam2Cache)));
        rearCamTagPub.set(getSeenTags(getCamTargets(rearCam, rearCamCache)));
    }

    Pose3d[] getSeenTags(List<Integer> seenTagIds) {
        List<Pose3d> seenTags = new ArrayList<>();
        var tags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField).getTags();
        for (int i = 0; i < seenTagIds.size(); i++) {
            var seenTagIdsArr = seenTagIds.toArray(Integer[]::new);
            seenTags.add(tags.get(seenTagIdsArr[i] - 1).pose);
        }
        return seenTags.toArray(Pose3d[]::new);
    }

    List<Integer> getCamTargets(PhotonCamera cam, List<Integer> cache) {
        var results = cam.getAllUnreadResults();
        if (!results.isEmpty()) {
            cache.clear();
            var t =
                    results.get(0).getTargets().stream()
                            .map(PhotonTrackedTarget::getFiducialId)
                            .toList();
            cache.addAll(t);
        }
        System.out.printf("%s tags: %s%n", cam.getName(), cache.toString());
        return cache;
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
