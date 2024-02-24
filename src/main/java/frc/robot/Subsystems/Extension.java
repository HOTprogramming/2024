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
double fullyExtended = 2.25;
double fullyExtendedAmp = 2;
double middlePoint = 1.11;
double extensionZero = 0;
double initialShooterPos;
double currentShooterPos;


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
    none;
}

public enum ExtensionPhaseAmp{
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
    none;
}

ExtensionPhaseTrap extendTrapPhase;
ExtensionPhaseAmp extendAmpPhase;

public Extension(RobotState robotState) {

    this.robotState = robotState;
    this.constants = robotState.getConstants().getExtensionConstants();

    extendMotor = new TalonFX(constants.EXTENSIONCAN, "drivetrain");
    spitter = new VictorSPX(constants.SPITTERCAN);

    extendMagic = new MotionMagicVoltage(0);

    extendPosition = extendMotor.getPosition();
    extendVelocity = extendMotor.getVelocity();
  

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

    public ExtensionPhaseTrap returnExtensionPhaseTrap(ExtensionPhaseTrap phaseTrap){
        extendTrapPhase = phaseTrap;
        return extendTrapPhase;
    }

    public ExtensionPhaseTrap getExtensionPhaseTrap(){
        return extendTrapPhase;
    }

    public ExtensionPhaseAmp returnExtensionPhaseAmp(ExtensionPhaseAmp phaseAmp){
        extendAmpPhase = phaseAmp;
        return extendAmpPhase;
    }

    public ExtensionPhaseAmp getExtensionPhaseAmp(){
        return extendAmpPhase;
    }

    @Override
    public void enabled(RobotCommander commander) {
        extendPosition.refresh(); 
        extendVelocity.refresh();


        if(commander.armCommanded() == ArmCommanded.trap){
            if(robotState.getExtendPos()<=middlePoint){
            returnExtensionPhaseAmp(ExtensionPhaseAmp.one);
            }
            else if(getExtensionPhaseTrap() == ExtensionPhaseTrap.one && robotState.getExtendPos()>(middlePoint - 0.1)){
            returnExtensionPhaseTrap(ExtensionPhaseTrap.two);
            }
            else if(getExtensionPhaseTrap() == ExtensionPhaseTrap.two && robotState.getBeamBreak() == false){
            returnExtensionPhaseTrap(ExtensionPhaseTrap.three);
            initialShooterPos = robotState.getShooterPos();
            } 
            else if(getExtensionPhaseTrap() == ExtensionPhaseTrap.three && currentShooterPos>50){
            returnExtensionPhaseTrap(ExtensionPhaseTrap.four);
            }
            else if(getExtensionPhaseTrap() == ExtensionPhaseTrap.four && robotState.getExtendPos() > (fullyExtended - 0.1)){
                returnExtensionPhaseTrap(ExtensionPhaseTrap.five);
            }



            if(getExtensionPhaseTrap() == ExtensionPhaseTrap.one){
                //command extension to middle position
                extendMotor.setControl(extendMagic.withPosition(middlePoint).withSlot(0));
                spitter.set(ControlMode.PercentOutput, 0);
            }
            if(getExtensionPhaseTrap() == ExtensionPhaseTrap.two){
                //spin shooter until beambreak is false, hold extension position
                robotState.setShooterOnAmpTrap(true);
                extendMotor.setControl(extendMagic.withPosition(middlePoint).withSlot(0));
                spitter.set(ControlMode.PercentOutput, 0);
            }
            if(getExtensionPhaseTrap() == ExtensionPhaseTrap.three){
                //start encoder counts and keep spinning shooter, spin spitter, hold extension position
                currentShooterPos = robotState.getShooterPos() - initialShooterPos;
                robotState.setShooterOnAmpTrap(true);
                extendMotor.setControl(extendMagic.withPosition(middlePoint).withSlot(0));
                spitter.set(ControlMode.PercentOutput, 0.1);
            }
            if(getExtensionPhaseTrap() == ExtensionPhaseTrap.four){
                //move extension to fully extended position
                extendMotor.setControl(extendMagic.withPosition(fullyExtended).withSlot(0));
            }
            if(getExtensionPhaseTrap() == ExtensionPhaseTrap.five){
                //spin spitter out, hold extension position
                extendMotor.setControl(extendMagic.withPosition(fullyExtended).withSlot(0));
                spitter.set(ControlMode.PercentOutput, -0.1);
            }
    
        }

        else if(commander.armCommanded() == ArmCommanded.amp){
            if(robotState.getExtendPos()<=middlePoint){
                returnExtensionPhaseAmp(ExtensionPhaseAmp.one);
                }
                else if(getExtensionPhaseAmp() == ExtensionPhaseAmp.one && robotState.getExtendPos()>(middlePoint - 0.1)){
                returnExtensionPhaseAmp(ExtensionPhaseAmp.two);
                }
                else if(getExtensionPhaseAmp() == ExtensionPhaseAmp.two && robotState.getBeamBreak() == false){
                returnExtensionPhaseAmp(ExtensionPhaseAmp.three);
                } 
                else if(getExtensionPhaseAmp() == ExtensionPhaseAmp.three && robotState.getShooterPos() > 50){
                returnExtensionPhaseAmp(ExtensionPhaseAmp.four);
                }
                else if(getExtensionPhaseAmp() == ExtensionPhaseAmp.four && robotState.getExtendPos() > (fullyExtendedAmp - 0.1)){
                    returnExtensionPhaseAmp(ExtensionPhaseAmp.five);
                }
    
    
                if(getExtensionPhaseAmp() == ExtensionPhaseAmp.one){
                    //command extension to middle position
                    extendMotor.setControl(extendMagic.withPosition(middlePoint).withSlot(0));
                    spitter.set(ControlMode.PercentOutput, 0);
                }
                if(getExtensionPhaseAmp() == ExtensionPhaseAmp.two){
                    //spin shooter until beambreak is false, hold extension position
                    robotState.setShooterOnAmpTrap(true);
                    extendMotor.setControl(extendMagic.withPosition(middlePoint).withSlot(0));
                    spitter.set(ControlMode.PercentOutput, 0);
                }
                if(getExtensionPhaseAmp() == ExtensionPhaseAmp.three){
                    //start encoder counts and keep spinning shooter, spin spitter, hold extension position
                    currentShooterPos = robotState.getShooterPos() - initialShooterPos;
                    robotState.setShooterOnAmpTrap(true);
                    extendMotor.setControl(extendMagic.withPosition(middlePoint).withSlot(0));
                    spitter.set(ControlMode.PercentOutput, 0.1);
                }
                if(getExtensionPhaseTrap() == ExtensionPhaseTrap.four){
                    //move extension to fully extended position
                    extendMotor.setControl(extendMagic.withPosition(fullyExtendedAmp).withSlot(0));
                }
                if(getExtensionPhaseTrap() == ExtensionPhaseTrap.five){
                    //spin spitter out, hold extension position
                    extendMotor.setControl(extendMagic.withPosition(fullyExtendedAmp).withSlot(0));
                    spitter.set(ControlMode.PercentOutput, -0.1);
                }
        }

        else{
            returnExtensionPhaseTrap(ExtensionPhaseTrap.none);
            returnExtensionPhaseAmp(ExtensionPhaseAmp.none);
            extendMotor.setControl(extendMagic.withPosition(0).withSlot(0));
            spitter.set(ControlMode.PercentOutput, 0);
        }


        
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
