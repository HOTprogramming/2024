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
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm implements SubsystemBase {

RobotState robotState;    
TalonFX armMotor;
double armPosition;

MotionMagicVoltage armMagic;

public Arm(RobotState robotState) {
    this.robotState = robotState;
    armMotor = new TalonFX(ARM_CAN);

   armMagic = new MotionMagicVoltage(0);

  //  armMotor.roto

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

        armMotor.setControl(armMagic.withPosition(commander.armPosition1()).withSlot(0));


      double armPosition = commander.armPosition1();

      SmartDashboard.putNumber("Desired Arm Position", armPosition);
      StatusSignal<Double> armPos = armMotor.getPosition();
      SmartDashboard.putNumber("ArmPos", armPos.getValueAsDouble());
      SmartDashboard.putNumber("Joystick", armPosition);
    }
    public void disabled(){
        armMotor.stopMotor();
    }
    public void reset(){
        armMotor.stopMotor();
        armMotor.setPosition(0);
    }
}
