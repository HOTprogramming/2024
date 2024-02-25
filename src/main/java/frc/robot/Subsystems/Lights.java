package frc.robot.Subsystems;

import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.RobotCommander;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;


public class Lights implements SubsystemBase {
    ConstantsBase.Lights constants;
    RobotState robotState;
    CANdle candle;

    public Lights(RobotState robotState) { 
        this.robotState = robotState;
        constants = robotState.getConstants().getLightsConstants();
        candle = new CANdle(constants.LIGHTS_CAN, "rio");

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll);
    }


    @Override
    public void updateState() {
    }
    
    @Override
    public void enabled(RobotCommander commander){
        if (robotState.getBeamBreak()) {
            candle.setLEDs(255, 165, 0);  // orange
        } 
        else {
            candle.setLEDs(0, 0, 0);  // off
        }
    }
    

    @Override
    public void disabled() {
        if (robotState.getTwoTags()) {
            candle.setLEDs(0, 255, 0);  // green
        } 
        else if (robotState.getOneTag()) {
            candle.setLEDs(255, 255, 0);  // yellow
        }
        else if (robotState.getNoTag()){
            candle.setLEDs(255, 0, 0);  // red
        }
        else {
            candle.setLEDs(255, 255, 255);  // white
        }
    }

    @Override
    public void reset() {
    }

    @Override
    public void init(RobotCommander commander) {
    }
}