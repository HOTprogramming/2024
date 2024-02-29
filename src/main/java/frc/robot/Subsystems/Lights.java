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
    CANdle candleRight;
    CANdle candleLeft;

    public Lights(RobotState robotState) { 
        this.robotState = robotState;
        constants = robotState.getConstants().getLightsConstants();
        candleRight = new CANdle(constants.LIGHTS_CAN_RIGHT, "rio");
        candleLeft = new CANdle(constants.LIGHTS_CAN_LEFT, "rio");

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candleRight.configAllSettings(configAll);
        candleLeft.configAllSettings(configAll);
    }


    @Override
    public void updateState() {
    }

    private void setLEDs(int r, int g, int b) {
        candleRight.setLEDs(r, g, b);
        candleLeft.setLEDs(r, g, b);
    }
    
    @Override
    public void enabled(RobotCommander commander){
        if (robotState.getBeamBreak()) {
            setLEDs(255, 165, 0);  // orange
        } 
        else {
            setLEDs(0, 0, 0);  // off
        }
    }
    

    @Override
    public void disabled() {
        if (robotState.getTwoTags()) {
            setLEDs(0, 255, 0);  // green
        } 
        else if (robotState.getOneTag()) {
            setLEDs(255, 255, 0);  // yellow
        }
        else if (robotState.getNoTag()){
            setLEDs(255, 0, 0);  // red
        }
        else {
            setLEDs(255, 255, 255);  // white
        }
    }

    @Override
    public void reset() {
    }

    @Override
    public void init(RobotCommander commander) {
    }
}