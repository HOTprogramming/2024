package frc.robot.Subsystems;

import static frc.robot.Constants.ArmConstants.*;

import frc.robot.RobotCommander;
import frc.robot.RobotState;

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
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm implements SubsystemBase {

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

public Arm(RobotState robotState) {
    this.robotState = robotState;
    armMotor = new TalonFX(ARM_CAN);
    cancoder = new CANcoder(1);

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
    mm.MotionMagicCruiseVelocity = CRUISEVELOCITY; //rps
    mm.MotionMagicAcceleration = ACCELERATION;
    mm.MotionMagicJerk = JERK;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = ARMKP;
    slot0.kI = ARMKI;
    slot0.kD = ARMKD;
    slot0.kV = ARMKV;
    slot0.kS = ARMKS; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8;


    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = 0.4;
    cancoder.getConfigurator().apply(cancoderConfig);

    cfg.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.SensorToMechanismRatio = 1.0;
    cfg.Feedback.RotorToSensorRatio = 12.8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = armMotor.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
    
}  

    public void updateState(){
        robotState.setArmPos(armMotor.getPosition().getValueAsDouble());
    }


    public void enabled(RobotCommander commander){

      armPosition.refresh(); 
      armVelocity.refresh();
      cancoderPosition.refresh(); 
      cancoderVelocity.refresh();

      armMotor.setControl(armMagic.withPosition(commander.armPosition1()).withSlot(0));


      double armComPosition = commander.armPosition1();

      SmartDashboard.putNumber("Desired Arm Position", armComPosition);

      SmartDashboard.putNumber("ArmPos", armPosition.getValueAsDouble());
      SmartDashboard.putNumber("ArmVelocity", armVelocity.getValueAsDouble());
    }
    public void disabled(){
        armMotor.stopMotor();
    }
    public void reset(){
        armMotor.stopMotor();
        armMotor.setPosition(0);
    }
}
