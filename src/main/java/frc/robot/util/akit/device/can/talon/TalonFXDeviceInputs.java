package frc.robot.util.akit.device.can.talon;

import frc.robot.util.akit.device.PhysicalDeviceInputs;
import frc.robot.util.akit.device.can.talon.inputs.TalonFXMotorInputsAutoLogged;
import frc.robot.util.akit.device.can.talon.inputs.TalonFXPIDInputsAutoLogged;
import frc.robot.util.akit.device.can.talon.inputs.TalonFXPositionInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TalonFXDeviceInputs extends PhysicalDeviceInputs {
    public TalonFXPositionInputsAutoLogged positionInputs = new TalonFXPositionInputsAutoLogged();
    public TalonFXPIDInputsAutoLogged pidInputs = new TalonFXPIDInputsAutoLogged();
    public TalonFXMotorInputsAutoLogged motorInputs = new TalonFXMotorInputsAutoLogged();
}
