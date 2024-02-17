package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import static frc.robot.Constants.ExtensionConstants.*;

public class Extension implements SubsystemBase{

RobotState robotState;  
TalonFX extendMotor;

MotionMagicVoltage extendMagic;

StatusSignal<Double> extendPosition;
StatusSignal<Double> extendVelocity;

Victor spitter;

public Extension(RobotState robotState) {

    this.robotState = robotState;
    extendMotor = new TalonFX(EXTENSION_CAN, "drivetrain");
    spitter = new Victor(SPITTER_CAN);

    extendMagic = new MotionMagicVoltage(0);

    extendPosition = extendMotor.getPosition();
    extendVelocity = extendMotor.getVelocity();
  

}

    public void extensionInit(){
    TalonFXConfiguration ecfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs emm = ecfg.MotionMagic;
    emm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
    emm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    emm.MotionMagicJerk = 50;

    Slot0Configs eSlot0 = ecfg.Slot0;
    eSlot0.kP = 60;
    eSlot0.kI = 0;
    eSlot0.kD = 0.1;
    eSlot0.kV = 0.12;
    eSlot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = ecfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = extendMotor.getConfigurator().apply(ecfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
    }



    @Override
    public void updateState() {
        
        robotState.setExtendPos(extendPosition.getValueAsDouble()*360);
    }

    @Override
    public void init(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    @Override
    public void enabled(RobotCommander commander) {
        extendPosition.refresh(); 
        extendVelocity.refresh();

       

        if(commander.extend() && robotState.getExtendPos()<=4){
        extendMotor.setControl(extendMagic.withPosition(4).withSlot(0));
        spitter.setVoltage(0);
        }
        else if(commander.extend() && robotState.getExtendPos()>4 && robotState.getShooterPos()<50){
        spitter.setVoltage(0.1);
        }
        else if (commander.extend() && robotState.getShooterPos()>=50){
        extendMotor.setControl(extendMagic.withPosition(7).withSlot(0));
        spitter.setVoltage(0);
        }
        else if(commander.extend() && robotState.getExtendPos()>=6.9){
        extendMotor.setControl(extendMagic.withPosition(7).withSlot(0));
        spitter.setVoltage(0.1);
        }
        else{
        extendMotor.setControl(extendMagic.withPosition(0).withSlot(0));
        spitter.setVoltage(0);
        }


    }

    @Override
    public void disabled() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'disabled'");
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }
    
}
