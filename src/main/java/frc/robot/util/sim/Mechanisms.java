package frc.robot.util.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

@Logged
public class Mechanisms {
    public Mechanism2d elevatorArmMech;

    private final MechanismLigament2d liftLigament;
    private final MechanismLigament2d armLigament;

    private final StructArrayPublisher<Pose3d> realComponentPosePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("ComponentPoses/Real", Pose3d.struct)
                    .publish();
    private final StructArrayPublisher<Pose3d> targetComponentPosePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("ComponentPoses/Target", Pose3d.struct)
                    .publish();

    public Mechanisms() {
        elevatorArmMech = new Mechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(100));

        liftLigament =
                elevatorArmMech
                        .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                        .append(
                                new MechanismLigament2d(
                                        "lift",
                                        Units.feetToMeters(3),
                                        90,
                                        6,
                                        new Color8Bit(Color.kRed)));
        elevatorArmMech
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(
                        new MechanismLigament2d(
                                "bottom",
                                Units.feetToMeters(3),
                                0,
                                6,
                                new Color8Bit(Color.kGreen)));
        armLigament =
                liftLigament.append(
                        new MechanismLigament2d(
                                "arm", Units.inchesToMeters(12), 0, 6, new Color8Bit(Color.kBlue)));
    }

    @Logged(name = "CoralManipulators")
    public Mechanism2d getCoralManipulatorMech() {
        return elevatorArmMech;
    }

    public void updateElevatorArmMech(Angle elevatorPos, Angle armPos) {
        liftLigament.setLength(Units.inchesToMeters((elevatorPos.magnitude() * 6) + 1));
        armLigament.setAngle(armPos.in(Degrees) - 90);

        SmartDashboard.putData("Mechanisms/CoralManipulator", elevatorArmMech);
    }

    public void publishComponentPoses(
            Angle elevatorPos, Angle armPos, Angle wristPos, boolean useRealPoses) {
        double elevatorStageHeight = Units.inchesToMeters(elevatorPos.times(8.6).magnitude());
        double carriageHeight = Units.inchesToMeters(elevatorPos.times(15).magnitude());
        double armAngle = armPos.in(Radians);
        double wristAngle = wristPos.in(Radians);

        Pose3d armPose =
                new Pose3d(
                        Units.inchesToMeters(-2.81),
                        0,
                        Units.inchesToMeters(10.22) + carriageHeight,
                        new Rotation3d(armAngle, 0, 0));
        Pose3d grabberPose =
                new Pose3d(0, Units.inchesToMeters(14.76), 0, new Rotation3d(0, wristAngle, 0));

        (useRealPoses ? realComponentPosePublisher : targetComponentPosePublisher)
                .set(
                        new Pose3d[] {
                            new Pose3d(
                                    Units.inchesToMeters(-8),
                                    0.0,
                                    Units.inchesToMeters(2.625) + elevatorStageHeight,
                                    Rotation3d.kZero),
                            new Pose3d(
                                    Units.inchesToMeters(-4.13),
                                    0,
                                    Units.inchesToMeters(10.22) + carriageHeight,
                                    Rotation3d.kZero),
                            armPose,
                            armPose.transformBy(
                                    new Transform3d(
                                            grabberPose.getTranslation(),
                                            grabberPose.getRotation()))
                        });
    }
}
