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
import com.ctre.phoenix6.configs.Slot1Configs;
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
double fullyExtended = 2.16;
double fullyExtendedAmp = 1.08;
double middlePoint = 0.6;
double extensionZero = 0;
double initialShooterPos;
double currentShooterPos;
double ampShooterPose = 0;
double ampCurrentShooterPose = 0;
double extensionTimer;
double extendedCommandedPosition;
boolean ampCycle = false;


public enum ExtensionPhaseTrap{
    one,
    two,
    three,
    four,
    five,
    six,
    seven,
    eight,
    nine,
    ten,
    none,
    timer,
    driver,
    stop;
}

ExtensionPhaseTrap extendTrapPhase;

public Extension(RobotState robotState) {

    this.robotState = robotState;
    this.constants = robotState.getConstants().getExtensionConstants();

    extendMotor = new TalonFX(constants.EXTENSIONCAN, "drivetrain");
    spitter = new VictorSPX(robotState.getConstants().getIntakeConstants().SLURPER_ROLLER_CAN);

    extendMagic = new MotionMagicVoltage(0);

    extendPosition = extendMotor.getPosition();
    extendVelocity = extendMotor.getVelocity();
  

}

    public void extensionInit(){
    TalonFXConfiguration ecfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs emm = ecfg.MotionMagic;
    emm.MotionMagicCruiseVelocity = 70; // 5 rotations per second cruise
    emm.MotionMagicAcceleration = 70; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    emm.MotionMagicJerk = 120;

    Slot0Configs eSlot0 = ecfg.Slot0;
    eSlot0.kP = constants.EKP;
    eSlot0.kI = constants.EKI;
    eSlot0.kD = constants.EKD;
    eSlot0.kV = constants.EKV;
    eSlot0.kS = constants.EKS; // Approximately 0.25V to get the mechanism moving

    Slot1Configs eSlot1 = ecfg.Slot1;
    eSlot1.kP = 45;
    eSlot1.kI = 0.0;
    eSlot1.kD = 0;
    eSlot1.kV = 0.0;
    eSlot1.kS = 0.0; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = ecfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8;
    ecfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
    returnExtensionPhaseTrap(ExtensionPhaseTrap.none);
    extensionTimer = 0;
    }



    @Override
    public void updateState() {
        extendPosition.refresh(); 
        extendVelocity.refresh();

        robotState.setExtendPos(extendPosition.getValueAsDouble());
        SmartDashboard.putNumber("extensionPos", extendPosition.getValueAsDouble());
        SmartDashboard.putNumber("extensionzero", 0);
        SmartDashboard.putNumber("extensiontrap", fullyExtended);
        SmartDashboard.putString("extensionenum", getExtensionPhaseTrap().toString());
    }

    @Override
    public void init(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    public void returnExtensionPhaseTrap(ExtensionPhaseTrap phaseTrap){
        this.extendTrapPhase = phaseTrap;
    }

    public ExtensionPhaseTrap getExtensionPhaseTrap(){
        return this.extendTrapPhase;
    }


    @Override
    public void teleop(RobotCommander commander) {
        extendPosition.refresh(); 
        extendVelocity.refresh();
        
        if(commander.armCommanded() == ArmCommanded.handoff){
            SmartDashboard.putNumber("shooterposextensionclass", robotState.getShooterPos());
            if(extensionTimer < 87){
            returnExtensionPhaseTrap(ExtensionPhaseTrap.one);
            SmartDashboard.putNumber("firststage", 1);
            }
            else if(getExtensionPhaseTrap() == ExtensionPhaseTrap.one && extendPosition.getValueAsDouble() > (middlePoint - 0.05)){
            returnExtensionPhaseTrap(ExtensionPhaseTrap.two);
            SmartDashboard.putNumber("thirdstage", 1);
            }
            else if(getExtensionPhaseTrap() == ExtensionPhaseTrap.two && robotState.getBeamBreak() == false){
            returnExtensionPhaseTrap(ExtensionPhaseTrap.three);
            SmartDashboard.putNumber("initialshooterpos", initialShooterPos);
            SmartDashboard.putNumber("fifthstage", 1);
            } 
            else if(getExtensionPhaseTrap() == ExtensionPhaseTrap.three && currentShooterPos < constants.SHOOTERENCODER){
            returnExtensionPhaseTrap(ExtensionPhaseTrap.three);
            SmartDashboard.putNumber("seventhstage", 1);
            }
            else if(getExtensionPhaseTrap() == ExtensionPhaseTrap.three && currentShooterPos > constants.SHOOTERENCODER){
            returnExtensionPhaseTrap(ExtensionPhaseTrap.driver);
            SmartDashboard.putNumber("andreas", 1);
            }
            else if(getExtensionPhaseTrap() == ExtensionPhaseTrap.driver){
            returnExtensionPhaseTrap(ExtensionPhaseTrap.driver);
            SmartDashboard.putNumber("ninthstage", 1);
            }

            if(getExtensionPhaseTrap() == ExtensionPhaseTrap.one){
                //command extension to middle position
                if(extensionTimer>12){
                robotState.setShooterOnAmpTrap(true);
                robotState.setFeederOnAmpTrap(false);
                extendMotor.setControl(extendMagic.withPosition(middlePoint).withSlot(0));
                spitter.set(ControlMode.PercentOutput, 0.8);
                }
                SmartDashboard.putNumber("secondstage", 1);
                extensionTimer++;
            }
            if(getExtensionPhaseTrap() == ExtensionPhaseTrap.two){
                //spin shooter until beambreak is false, hold extension position
                robotState.setShooterOnAmpTrap(true);
                robotState.setFeederOnAmpTrap(true);
                extendMotor.setControl(extendMagic.withPosition(middlePoint).withSlot(0));
                initialShooterPos = robotState.getShooterPos();
                spitter.set(ControlMode.PercentOutput, 0.8);
                SmartDashboard.putNumber("fourthstage", 1);
            }
            if(getExtensionPhaseTrap() == ExtensionPhaseTrap.three){
                //start encoder counts and keep spinning shooter, spin spitter, hold extension position
                currentShooterPos = robotState.getShooterPos() - initialShooterPos;
                robotState.setShooterOnAmpTrap(true);
                extendMotor.setControl(extendMagic.withPosition(middlePoint).withSlot(0));
                spitter.set(ControlMode.PercentOutput, 0.8);
                SmartDashboard.putNumber("sixthstage", 1);
                SmartDashboard.putNumber("currentshooterpos", currentShooterPos);
            }
            if(getExtensionPhaseTrap() == ExtensionPhaseTrap.driver){
                //move extension to fully extended position
                extendMotor.setControl(extendMagic.withPosition(middlePoint).withSlot(0));
                robotState.setShooterOnAmpTrap(false);
                robotState.setFeederOnAmpTrap(false);
                spitter.set(ControlMode.PercentOutput, 0);
                SmartDashboard.putNumber("driverstage", 1);
            }
        }

        else if(commander.armCommanded() == ArmCommanded.trap){
            SmartDashboard.putNumber("here", 1);
            extendedCommandedPosition = fullyExtended;
            spitter.set(ControlMode.PercentOutput, 0);
            extendMotor.setControl(extendMagic.withPosition(fullyExtended).withSlot(0));
            SmartDashboard.putNumber("extendedCommandedPosition", extendedCommandedPosition);
        }

        else{
            returnExtensionPhaseTrap(ExtensionPhaseTrap.none);
            if (commander.extensionOveride()) {
                extendMotor.set(-0.2);

            } else if(extendPosition.getValueAsDouble() > 0.30) { 
                extendMotor.set(-0.50); //.30

            } else if (extendPosition.getValueAsDouble() > 0.1){
                
                extendMotor.set(-0.25);
            
            } else if (extendPosition.getValueAsDouble() > 0.0045){
                extendMotor.set(-0.20);

            } else {
                extendMotor.setControl(extendMagic.withPosition(0).withSlot(0));
            }

            if(extendPosition.getValueAsDouble() > 0.5){
                robotState.setArmOnAmpRetract(true);
                }
                else{
                robotState.setArmOnAmpRetract(false);
                }

            if (!commander.getIntake()) {
                spitter.set(ControlMode.PercentOutput, 0);
            }

            extensionTimer = 0;
        }

        if(commander.setShoot() && (commander.armCommanded() == ArmCommanded.amp || 
                                    commander.armCommanded() == ArmCommanded.handoff ||
                                    commander.armCommanded() == ArmCommanded.trap)){
            spitter.set(ControlMode.PercentOutput, -0.8);
        }

        if (commander.extensionZero()) {
            extendMotor.setPosition(-0.06);
        }

        if (commander.armCommanded() == ArmCommanded.amp) {
            extendMotor.setControl(extendMagic.withPosition(0.89).withSlot(0));
            ampCurrentShooterPose = robotState.getShooterPos();

            if (commander.setShoot() && (ampCurrentShooterPose - ampShooterPose < 7)) {
                spitter.set(ControlMode.PercentOutput, 1);
            } else if (commander.setShoot() && (ampCurrentShooterPose - ampShooterPose >= 7)) { // could be implied
                spitter.set(ControlMode.PercentOutput, -1);
                
            } else if(robotState.getBeamBreak() == true){
                ampShooterPose = robotState.getShooterPos();
                spitter.set(ControlMode.PercentOutput, 1);
            }
            else{
                spitter.set(ControlMode.PercentOutput, 0);  
            }
        }

        if (commander.trapArmFineControl() == 1) {
            spitter.set(ControlMode.PercentOutput, 0.3);
        } else if (commander.trapArmFineControl() == -1) {
            spitter.set(ControlMode.PercentOutput, -0.3);
        } 

    }

    @Override
    public void cameraLights() {
       extendMotor.stopMotor();
    }

    @Override
    public void reset() {
        extendMotor.stopMotor();
    }
    
}
