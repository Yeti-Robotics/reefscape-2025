package frc.robot.util.sim;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

@Logged
public class Mechanisms {
    public Mechanism2d elevatorArmMech;

    private MechanismLigament2d liftLigament;
    private MechanismLigament2d armLigament;

    private StructArrayPublisher<Pose3d> componentPosePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("ComponentPoses", Pose3d.struct)
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

    public void updateElevatorArmMech(double elevatorPos, double armPos) {
        liftLigament.setLength(elevatorPos);
        armLigament.setAngle(armPos);
        SmartDashboard.putData("Mechanisms/CoralManipulator", elevatorArmMech);
        publishComponentZeroes();
    }

    private void publishComponentZeroes() {
        componentPosePublisher.set(new Pose3d[] {Pose3d.kZero, Pose3d.kZero, Pose3d.kZero});
    }
}
