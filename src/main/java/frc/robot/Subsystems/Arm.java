package frc.robot.Subsystems;

import static frc.robot.Constants.ArmConstants.*;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import frc.robot.utils.InterpolatingDouble;

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

import frc.robot.utils.InterpolatingTreeMap;
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


static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> armMap = new InterpolatingTreeMap<>();


// public enum armDesiredPos{

//   shoot(127),
//   zero(95);

//   public final double armcpos;
//   public double getcommmPosition(){
//     return armcpos;
//   }

// armDesiredPos(double armcpos){
// this.armcpos = armcpos;
// }


// }

public boolean setArmDesPos;
public double armComPos;

public Arm(RobotState robotState) {
    this.robotState = robotState;
    armMotor = new TalonFX(ARM_CAN, "drivetrain");
    cancoder = new CANcoder(CANCODER_CAN, "drivetrain");               

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

  armMap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(2.0));
    armMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(3.0));

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
    fdb.SensorToMechanismRatio = 1;


    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = 0.4;
    cancoder.getConfigurator().apply(cancoderConfig);

    cfg.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.SensorToMechanismRatio = 1; //changes what the cancoder and fx encoder ratio is
    cfg.Feedback.RotorToSensorRatio = 4096/360; //12.8;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //cancoder.setPosition(0);


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
        robotState.setArmPos(armMotor.getPosition().getValueAsDouble());
    }


    double mapArmPos = ZERO;

    public void enabled(RobotCommander commander){

      armPosition.refresh(); 
      armVelocity.refresh();
      cancoderPosition.refresh(); 
      cancoderVelocity.refresh();

      //armDesiredPos thePos = commander.armPosition();

      //armMotor.setControl(armMagic.withPosition(thePos.getcommmPosition()/360).withSlot(0));
      
      if(commander.runArm()){
        armComPos = SHOOT/360;
        armMotor.setControl(armMagic.withPosition(armComPos).withSlot(0));

        mapArmPos = armMap.getInterpolated(new InterpolatingDouble(1.5)).value;
      
      } else if (commander.zeroArm()) {
        armComPos = ZERO/360;
         armMotor.setControl(armMagic.withPosition(armComPos).withSlot(0));

      } else{
        armMotor.setVoltage(0);
      }

      SmartDashboard.putNumber("Cancoder", cancoderPosition.getValueAsDouble()*360);
      SmartDashboard.putNumber("CancoderVelocity", cancoderVelocity.getValueAsDouble());
      SmartDashboard.putNumber("Calced Arm Pose", mapArmPos);
      SmartDashboard.putNumber("ArmPos", armPosition.getValueAsDouble()*360);
      SmartDashboard.putNumber("ArmVelocity", armVelocity.getValueAsDouble()*360);
      SmartDashboard.putNumber("ArmCommandedPosition", armComPos*360);
      //SmartDashboard.putNumber("ArmCommandedPosition", thePos.getcommmPosition());

    }
    public void disabled(){
        armMotor.stopMotor();
    }
    public void reset(){
        armMotor.stopMotor();
        armMotor.setPosition(0);
    }

    @Override
    public void init(RobotCommander commander) {
      // TODO Auto-generated method stub
    }
}
