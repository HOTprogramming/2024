package frc.robot.Subsystems;

import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.RobotCommander;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Climber implements SubsystemBase {
    ConstantsBase.Climber constants;
    RobotState robotState;
    TalonFX climberMotor;

    public Climber(RobotState robotState) { 
        this.robotState = robotState;
        constants = robotState.getConstants().getClimberConstants();
        climberMotor = new TalonFX(constants.CLIMBER_CAN, "drivetrain");
        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
        climberConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberMotor.getConfigurator().apply(climberConfigs);
    }


    @Override
    public void updateState() {
    }
    
    @Override
    public void enabled(RobotCommander commander) {
        if (commander.climberUp()) {
            climberMotor.set(constants.CLIMBER_SPEED);
        } else if (commander.climberDown()) {
            climberMotor.set(-constants.CLIMBER_SPEED);
        } else {
            climberMotor.set(0);
        }
        SmartDashboard.putBoolean("ClimbUp", commander.climberUp());
        SmartDashboard.putBoolean("ClimbDown", commander.climberDown());
    }
    

    @Override
    public void disabled() {
    }

    @Override
    public void reset() {
    }

    @Override
    public void init(RobotCommander commander) {
    }
}