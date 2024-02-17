package frc.robot.Subsystems;

import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.RobotCommander;
import com.ctre.phoenix.led.CANdle;


public class Lights implements SubsystemBase {
    ConstantsBase.Lights constants;
    RobotState robotState;
    CANdle leds = new CANdle(constants.LIGHTS_CAN, "drivetrain");


    public Lights(RobotState robotState) { 
        this.robotState = robotState;
        constants = robotState.getConstants().getLightsConstants();

    }


    @Override
    public void updateState() {
    }
    
    @Override
    public void enabled(RobotCommander commander){
    }
    

    @Override
    public void disabled() {
    }

    @Override
    public void reset() {
    }


    @Override
    public void init(RobotCommander commander) {
    }
}