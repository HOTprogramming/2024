package frc.robot.Subsystems;

import frc.robot.RobotState;
import frc.robot.ConstantsFolder.CamBotConstants;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.Subsystems.Camera.CameraPositions;
import frc.robot.RobotCommander;

import java.util.List;
import java.util.Map;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import org.photonvision.targeting.PhotonTrackedTarget;


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

    private void setCameraLEDR(Map<CameraPositions, List<PhotonTrackedTarget>> list, CameraPositions key, CANdle caNdle, int start, int count) {
        int size = -1;

        if (list == null) {
            size = -1;
        } else if (list.containsKey(key)) {
            size = list.get(key).size();
        } else {
            size = 0;
        }
        
        if (size >= 3) {
            caNdle.setLEDs(0, 0, 255, 0, start, count);  // blue
        } 
        else if (size == 2) {
            caNdle.setLEDs(0, 255, 0, 0, start, count);  // green
        } 
        else if (size == 1) {
            caNdle.setLEDs(255, 255, 0, 0, start, count);  // yellow
        }
        else if (size == 0){
            caNdle.setLEDs(255, 0, 0, 0, start, count);  // red
        }
        else {
            caNdle.setLEDs(255, 255, 255, 0, start, count);  // white
        }
    }
    
    @Override
    public void teleop(RobotCommander commander){
        if (robotState.getBeamBreak()) {
            candleLeft.setLEDs(255, 165, 0, 0, 1,6);  // orange
            candleLeft.setLEDs(255, 165, 0, 0, 11 ,16);  // orange
            candleRight.setLEDs(255, 165, 0, 0, 1,6);  // orange
            candleRight.setLEDs(255, 165, 0, 0, 11 ,20);  // orange
        } 
        else {
            candleLeft.setLEDs(0, 0, 0, 0, 1,6);  // off
            candleLeft.setLEDs(0, 0, 0, 0, 11 ,16);  // off
            candleRight.setLEDs(0, 0, 0, 0, 1,6);  // off
            candleRight.setLEDs(0, 0, 0, 0, 11 ,20);  // off
        }
        
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.FRONT,candleLeft,8,3);
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.FRONT,candleLeft,27,3);
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.RIGHT,candleLeft,0,1);
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.RIGHT,candleLeft,7,1);
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.BACK,candleRight,8,3);
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.BACK,candleRight,31,3);
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.LEFT,candleRight,0,1);
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.LEFT,candleRight,7,1);
    }
    
    @Override
    public void cameraLights() {
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.RIGHT,candleRight,0,8);
        
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.FRONT,candleLeft,8,22);

        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.BACK,candleRight,8,27);
 
        setCameraLEDR(robotState.getTargetsSeenByCamera(),CameraPositions.LEFT,candleLeft,0,8);
    }

    @Override
    public void reset() {
    }

    @Override
    public void init(RobotCommander commander) {
    }
}