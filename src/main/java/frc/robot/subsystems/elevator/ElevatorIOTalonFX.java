package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.PhysicsSim;

public class ElevatorIOTalonFX implements ElevatorIO{
    private final TalonFX primaryMotor;
    private final TalonFX secondaryMotor;

    public ElevatorIOTalonFX() {
        // initialize motors from config
        primaryMotor = new TalonFX(ElevatorConfig.primaryMotorID/*, Constants.RIO_BUS*/);
        secondaryMotor = new TalonFX(ElevatorConfig.secondaryMotor/*taht abv code should add the motor to a can bus buuuuut it kinda doesnt work /shrug*/);
        // this is bsicallt the whhooolllee sim io file thing right here bc im so cool and better than tjaowi
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(primaryMotor);
            PhysicsSim.getInstance().addTalonFX(secondaryMotor);
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.positionRotation = primaryMotor.getPosition().getValueAsDouble();
        inputs.velocityRPM = primaryMotor.getVelocity().getValueAsDouble();
    }

    // aaa make it work pls i need to leave this place and commit
//    @Override
//    public void moveTo(Angle pos) {
//        primaryMotor.setControl(magicRequest.with)
//    }
}
