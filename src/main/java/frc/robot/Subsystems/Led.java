package frc.robot.Subsystems;

import frc.robot.RobotState;

import frc.robot.RobotCommander;

import static frc.robot.Constants.CANdleConstants.*;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;


public class Led implements SubsystemBase{
    private CANdle candle;
    RobotState robotState; 
    private final int LEDOFFSET = 8; 
    private final int LedCount = 8;

    public Led(RobotState robotstate){ 
    this.robotState = robotState;
    {
        candle = new CANdle(LED_CAN);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.5;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll);
    
}
     
    

}
    @Override
    public void updateState(){


    }

    @Override
    public void enabled(RobotCommander commander){
        candle.setLEDs(50, 16, 105);
       
    }
    @Override
    public void disabled(){

    }
    @Override
    public void reset() {
    }
    
}

