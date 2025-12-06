package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.PhysicsSim;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX climber;
    final MotionMagicTorqueCurrentFOC magicRequest;

    public ClimberIOTalonFX() {
        climber = new TalonFX(ClimberConfig.climberId, Constants.RIO_BUS);
        climber.getConfigurator().apply(ClimberConfig.climberTalonFXConfigs);

        magicRequest = new MotionMagicTorqueCurrentFOC(0);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(climber);
        }
    }

    @Override
    public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
        inputs.climberPosition = climber.getPosition().getValueAsDouble();
        inputs.climberVelocityRPM = climber.getVelocity().getValueAsDouble();
    }

    @Override
    public void setClimberSpeed(double speed) {
        climber.set(speed);
    }

    @Override
    public void stop() {
        climber.stopMotor();
    }

    @Override
    public void setClimberPosition(ClimberPosition position) {
        climber.setControl(magicRequest.withPosition(position.getAngle()));
    }
}
