package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Arm extends SubsystemBase {
    private final TalonFX armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
    private final MotionMagicTorqueCurrentFOC motionRequest = new MotionMagicTorqueCurrentFOC(0);

    public Arm() {
        armKraken.getConfigurator().apply(ArmConfig.armMotorConfig);
        CANcoder armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.CANIVORE_BUS);

        armEncoder.getConfigurator().apply(ArmConfig.armCanCoderConfig);
    }

    public Command moveToPosition(ArmPositions position) {
        return runOnce(() -> armKraken.setControl(motionRequest.withPosition(position.getAngle())));
    }
}
