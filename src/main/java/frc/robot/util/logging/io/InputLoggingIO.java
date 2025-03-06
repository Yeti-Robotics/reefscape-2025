package frc.robot.util.logging.io;

import frc.robot.util.device.impl.TalonFXMotor;

public interface InputLoggingIO<T> {
    void updateInputs(T inputs);

}
