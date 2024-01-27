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

}

  public void armInit(){
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 10; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    mm.MotionMagicJerk = 50;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 60;
    slot0.kI = 0;
    slot0.kD = 0.1;
    slot0.kV = 0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

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

        armMotor.setControl(armMagic.withPosition(commander.getTargetArmSpeed()).withSlot(0));


double armPosition = commander.getTargetArmSpeed();

      StatusSignal<Double> armPos = armMotor.getPosition();
      SmartDashboard.putNumber("ArmPos", armPos.getValueAsDouble());
      SmartDashboard.putNumber("Joystick", armPosition);
      StatusSignal<Double> armSpeed = armMotor.getVelocity();
      SmartDashboard.putNumber("ArmSpeed", armSpeed.getValueAsDouble());

    }
    public void disabled(){
        armMotor.stopMotor();
    }
    public void reset(){
        armMotor.stopMotor();
    }
}
