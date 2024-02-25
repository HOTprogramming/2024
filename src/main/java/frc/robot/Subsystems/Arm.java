package frc.robot.Subsystems;

import static frc.robot.Constants.ArmConstants.*;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import frc.robot.ShotMap;
import frc.robot.ConstantsFolder.ConstantsBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.utils.Interpolation.InterpolatingDouble;
import frc.robot.utils.Interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm implements SubsystemBase {

ConstantsBase.Arm constants;

RobotState robotState;    
TalonFX armMotor;

MotionMagicVoltage armMagic;

CANcoder cancoder;

StatusSignal<Boolean> f_fusedSensorOutOfSync;
StatusSignal<Boolean> sf_fusedSensorOutOfSync;
StatusSignal<Boolean> f_remoteSensorInvalid;
StatusSignal<Boolean> sf_remoteSensorInvalid;

StatusSignal<Double> armPosition;
StatusSignal<Double> armVelocity;
StatusSignal<Double> cancoderPosition;
StatusSignal<Double> cancoderVelocity;
StatusSignal<Double> armRotorPos;

public double robotePosToSpeaker;
public double commandedPosition;

ShotMap shotMap;

public enum ArmCommanded{
  shotMap,
  close,
  protect,
  amp,
  trap,
  trap2,
  handoff,
  trapZero,
  zero,
  auton,
  preload,
  none;
}

public Arm(RobotState robotState) {

    this.robotState = robotState;
    this.constants = robotState.getConstants().getArmConstants();

    shotMap = new ShotMap(robotState);
    armMotor = new TalonFX(constants.ARM_CAN, "drivetrain");
    cancoder = new CANcoder(constants.CANCODER_CAN, "drivetrain");               

    armMagic = new MotionMagicVoltage(0);

    f_fusedSensorOutOfSync = armMotor.getFault_FusedSensorOutOfSync();
    sf_fusedSensorOutOfSync = armMotor.getStickyFault_FusedSensorOutOfSync();
    f_remoteSensorInvalid = armMotor.getFault_RemoteSensorDataInvalid();
    sf_remoteSensorInvalid = armMotor.getStickyFault_RemoteSensorDataInvalid();

    armPosition = armMotor.getPosition();
    armVelocity = armMotor.getVelocity();
    cancoderPosition = cancoder.getPosition();
    cancoderVelocity = cancoder.getVelocity();
    armRotorPos = armMotor.getRotorPosition();


}

  public void armInit(){
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = constants.CRUISEVELOCITY; //rps
    mm.MotionMagicAcceleration = constants.ACCELERATION;
    mm.MotionMagicJerk = constants.JERK;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = constants.ARMKP;
    slot0.kI = constants.ARMKI;
    slot0.kD = constants.ARMKD;
    slot0.kV = constants.ARMKV;
    slot0.kS = constants.ARMKS; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 1;

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.ARMOFFSET;
    cancoder.getConfigurator().apply(cancoderConfig);

    cfg.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.SensorToMechanismRatio = 1; //changes what the cancoder and fx encoder ratio is
    cfg.Feedback.RotorToSensorRatio = 4096/360; //12.8;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .5;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = .25;


    StatusCode armStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      armStatus = armMotor.getConfigurator().apply(cfg);
      if (armStatus.isOK()) break;
    }
    if (!armStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + armStatus.toString());
    }
}  

    public void updateState(){
      robotePosToSpeaker = robotState.getPoseToSpeaker();

      robotState.setArmPos(armMotor.getPosition().getValueAsDouble());

      armPosition.refresh(); 
      armVelocity.refresh();
      cancoderPosition.refresh(); 
      cancoderVelocity.refresh();
      SmartDashboard.putNumber("Cancoder", cancoderPosition.getValueAsDouble()*360.0);
      SmartDashboard.putNumber("CancoderVelocity", cancoderVelocity.getValueAsDouble());
      SmartDashboard.putNumber("ArmPos", armPosition.getValueAsDouble()*360.0);
      SmartDashboard.putNumber("ArmPosRaw", armPosition.getValueAsDouble());
      SmartDashboard.putNumber("ArmVelocity", armVelocity.getValueAsDouble()*360.0);
      SmartDashboard.putNumber("posetospeaker", robotePosToSpeaker);  
      SmartDashboard.putNumber("armCommandedPosition", commandedPosition*360);
    }

    public void enabled(RobotCommander commander){

      armPosition.refresh(); 
      armVelocity.refresh();
      cancoderPosition.refresh(); 
      cancoderVelocity.refresh();
    

      if(commander.armCommanded() == ArmCommanded.shotMap){
        commandedPosition = shotMap.calcShotMap();
        SmartDashboard.putNumber("Arm_ShotmapPose", commandedPosition);
        if(commandedPosition >= 95.0){
        armMotor.setControl(armMagic.withPosition(commandedPosition/360.0).withSlot(0));
        }
      } 
       else if (commander.armCommanded() == ArmCommanded.zero) {
         commandedPosition = constants.ZERO/360.0;
         armMotor.setControl(armMagic.withPosition(commandedPosition).withSlot(0));
         
      } 
        else if(commander.armCommanded() == ArmCommanded.trap){
        commandedPosition = constants.TRAP/360.0;
        armMotor.setControl(armMagic.withPosition(commandedPosition).withSlot(0));

      }
      else if (commander.armCommanded() == ArmCommanded.close){
        commandedPosition = constants.CLOSE/360.0;
        armMotor.setControl(armMagic.withPosition(commandedPosition).withSlot(0));

      }
      else if (commander.armCommanded() == ArmCommanded.protect){
        commandedPosition = constants.PROTECT/360.0;
        armMotor.setControl(armMagic.withPosition(commandedPosition).withSlot(0));

      }
      else if (commander.armCommanded() == ArmCommanded.amp){
        commandedPosition = constants.AMP/360.0;
        armMotor.setControl(armMagic.withPosition(commandedPosition).withSlot(0));
      }
      else if (commander.armCommanded() == ArmCommanded.auton){
        commandedPosition = 121.5/360.0;
        armMotor.setControl(armMagic.withPosition(commandedPosition).withSlot(0));
      }
      else if (commander.armCommanded() == ArmCommanded.preload){
        commandedPosition = 143/360.0;
        armMotor.setControl(armMagic.withPosition(commandedPosition).withSlot(0));
      }
      else{
        armMotor.setVoltage(0);
      }
      SmartDashboard.putNumber("Cancoder", cancoderPosition.getValueAsDouble()*360.0);
      SmartDashboard.putNumber("CancoderVelocity", cancoderVelocity.getValueAsDouble());
      SmartDashboard.putNumber("ArmPos", armPosition.getValueAsDouble()*360.0);
      SmartDashboard.putNumber("ArmPosRaw", armPosition.getValueAsDouble());
      SmartDashboard.putNumber("ArmVelocity", armVelocity.getValueAsDouble()*360.0);
      SmartDashboard.putNumber("posetospeaker", robotePosToSpeaker);  
      SmartDashboard.putNumber("armCommandedPosition", commandedPosition);
    }
    public void disabled(){
        armMotor.stopMotor();
    }
    public void reset(){
        armMotor.stopMotor();
        armMotor.setPosition(0);
    }

    public void simulation(){
    }

    @Override
    public void init(RobotCommander commander) {
    }
}

