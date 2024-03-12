package frc.robot.Subsystems;

import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.RobotCommander;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Climber implements SubsystemBase {
    ConstantsBase.Climber constants;
    RobotState robotState;
    TalonFX climberMotor;

    MotionMagicVoltage climberMagic;

    StatusSignal<Double> climberPosition;

    public Climber(RobotState robotState) { 
        this.robotState = robotState;
        constants = robotState.getConstants().getClimberConstants();
        climberMotor = new TalonFX(constants.CLIMBER_CAN, "drivetrain");
        climberMagic = new MotionMagicVoltage(0);

        climberPosition = climberMotor.getPosition();

    }

    @Override
    public void updateState() {
        climberPosition.refresh();
        SmartDashboard.putNumber("climberposition", climberPosition.getValueAsDouble());
    }
    
    @Override
    public void teleop(RobotCommander commander) {
        // if (commander.climberUp()) {
        //     climberMotor.setControl(climberMagic.withPosition(constants.CLIMBERPOS).withSlot(0)); //125   
        // } else if (commander.climberDown() && climberPosition.getValueAsDouble() > 0) {
        //     climberMotor.setVoltage(-constants.CLIMBER_SPEED);
        // } else {
        //     climberMotor.setVoltage(0);
        // }

        if ((commander.climberUp() && climberPosition.getValueAsDouble() < 283) || (commander.climberUp() && commander.climberOverride()) ) {
            climberMotor.setVoltage(8);    
        } else if ((commander.climberDown() && climberPosition.getValueAsDouble() > 0) || (commander.climberDown() && commander.climberOverride())) {
            climberMotor.setVoltage(-8);
        } else {
            climberMotor.setVoltage(0);
        }


  
        SmartDashboard.putBoolean("ClimbUp", commander.climberUp());
        SmartDashboard.putBoolean("ClimbDown", commander.climberDown());
    }
    

    @Override
    public void cameraLights() {
    }

    @Override
    public void reset() {
    }

    @Override
    public void init(RobotCommander commander) {

        TalonFXConfiguration ccfg = new TalonFXConfiguration();

        /* Configure current limits */
        MotionMagicConfigs cmm = ccfg.MotionMagic;
        cmm.MotionMagicCruiseVelocity = 10; // 5 rotations per second cruise
        cmm.MotionMagicAcceleration = 5; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel 
        cmm.MotionMagicJerk = 50;
    
        Slot0Configs cSlot0 = ccfg.Slot0;
        cSlot0.kP = 5;
        cSlot0.kI = 0;
        cSlot0.kD = 0;
        cSlot0.kV = 2;
        cSlot0.kS = 2; // Approximately 0.25V to get the mechanism moving
    
        FeedbackConfigs cfdb = ccfg.Feedback;
        cfdb.SensorToMechanismRatio = 1;
        ccfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = climberMotor.getConfigurator().apply(ccfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure device. Error: " + status.toString());
        }

        climberMotor.setPosition(0);
    }
}