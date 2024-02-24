package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.Subsystems.Arm.ArmCommanded;

import static frc.robot.Constants.ExtensionConstants.*;

public class Extension implements SubsystemBase{

ConstantsBase.Extension constants;
RobotState robotState;  
TalonFX extendMotor;

MotionMagicVoltage extendMagic;
double extendedCommanded;

StatusSignal<Double> extendPosition;
StatusSignal<Double> extendVelocity;

VictorSPX spitter;

public Extension(RobotState robotState) {

    this.robotState = robotState;
    this.constants = robotState.getConstants().getExtensionConstants();

    extendMotor = new TalonFX(constants.EXTENSIONCAN, "drivetrain");
    spitter = new VictorSPX(constants.SPITTERCAN);

    extendMagic = new MotionMagicVoltage(0);

    extendPosition = extendMotor.getPosition();
    extendVelocity = extendMotor.getVelocity();
    //a
  

}

    public void extensionInit(){
    TalonFXConfiguration ecfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs emm = ecfg.MotionMagic;
    emm.MotionMagicCruiseVelocity = constants.ECRUISEVELOCITY; // 5 rotations per second cruise
    emm.MotionMagicAcceleration = constants.EACCELERATION; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    emm.MotionMagicJerk = constants.EJERK;

    Slot0Configs eSlot0 = ecfg.Slot0;
    eSlot0.kP = constants.EKP;
    eSlot0.kI = constants.EKI;
    eSlot0.kD = constants.EKD;
    eSlot0.kV = constants.EKV;
    eSlot0.kS = constants.EKS; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = ecfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8;
    ecfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = extendMotor.getConfigurator().apply(ecfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
    extendMotor.setPosition(0);
    spitter.setNeutralMode(NeutralMode.Brake);
    }



    @Override
    public void updateState() {
        extendPosition.refresh(); 
        extendVelocity.refresh();

        robotState.setExtendPos(extendPosition.getValueAsDouble());
        SmartDashboard.putNumber("extensionPos", extendPosition.getValueAsDouble());
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

        // if(commander.armCommanded() == ArmCommanded.trap && robotState.getExtendPos()<=1.11){
        // extendMotor.setControl(extendMagic.withPosition(1.11).withSlot(0));
        // spitter.setVoltage(0);
        // }
        // else if(commander.armCommanded() == ArmCommanded.trap && robotState.getExtendPos()>4 && robotState.getShooterPos()<50){
        // spitter.setVoltage(0.1);
        // }
        // else if (commander.armCommanded() == ArmCommanded.trap && robotState.getShooterPos()>=50){
        // extendMotor.setControl(extendMagic.withPosition(2.43).withSlot(0));
        // spitter.setVoltage(0);
        // }
        // else if(commander.armCommanded() == ArmCommanded.trap && robotState.getExtendPos()>=2.42){
        // extendMotor.setControl(extendMagic.withPosition(2.43).withSlot(0));
        // spitter.setVoltage(0.1);
        // }
        // else{
        // extendMotor.setControl(extendMagic.withPosition(0).withSlot(0));
        // spitter.setVoltage(0);
        // }
        SmartDashboard.putString("armCommanded", commander.armCommanded().toString());

        if(commander.armCommanded() == ArmCommanded.trap){
            extendedCommanded = 0.36;
            extendMotor.setControl(extendMagic.withPosition(extendedCommanded));
            SmartDashboard.putNumber("extendedCommanded", extendedCommanded);
        }
        else if(commander.armCommanded() == ArmCommanded.trap2){
            extendedCommanded = 2.25;
            extendMotor.setControl(extendMagic.withPosition(extendedCommanded));
            SmartDashboard.putNumber("extendedCommanded", extendedCommanded);
        }
        else if(commander.armCommanded() == ArmCommanded.trapZero){
            extendedCommanded = 0;
            extendMotor.setControl(extendMagic.withPosition(extendedCommanded));
            SmartDashboard.putNumber("extendedCommanded", extendedCommanded);
        }
        else{
        extendMotor.setVoltage(0);
        spitter.set(ControlMode.PercentOutput, 0.1);
        }


    }

    @Override
    public void disabled() {
       extendMotor.stopMotor();
    }

    @Override
    public void reset() {
        extendMotor.stopMotor();
    }
    
}
