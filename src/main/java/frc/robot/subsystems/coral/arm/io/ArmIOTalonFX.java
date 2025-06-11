package frc.robot.subsystems.coral.arm.io;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.util.akit.device.can.CANUtil;
import frc.robot.util.akit.device.can.cancoder.CANCoderDevice;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

import java.util.function.Supplier;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFXDevice armMotor = TalonFXDevice.configure(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS)
            .log("Arm/ArmMotor")
            .withConfig(ArmConfig.talonFXConfiguration)
            .modifyConfig(c -> c.MotorOutput.Inverted =
                    Robot.isReal() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
            .withFusedCANcoder(CANCoderDevice.configure(ArmConfig.ARM_CANCODER_ID, Constants.CANIVORE_BUS)
                    .log("Arm/CANcoder")
                    .withConfig(ArmConfig.cancoderConfiguration)
                    .syncConfigs()
                    .getDevice())
            .syncConfigs();

    private final Supplier<Angle> armMotorPosition = armMotor.positionSupplier(CANUtil.TALON_MAX_UPDATE_HZ);
    private final MotionMagicTorqueCurrentFOC motionMagicReq = new MotionMagicTorqueCurrentFOC(0);

    @Override
    public Angle getState() {
        return armMotorPosition.get();
    }

    @Override
    public void setState(Angle value) {
        armMotor.getDevice().setControl(motionMagicReq.withPosition(value));
    }
}
