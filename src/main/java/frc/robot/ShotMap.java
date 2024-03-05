package frc.robot;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;


public class ShotMap {
    ConstantsBase.Arm constants;

    RobotState robotState; 
    private double redDistance1;
    private double redDistance2;
    private double redDistance3;
    private double redDistance4;
    private double redDistance5;
    private double redAngle1;
    private double redAngle2;
    private double redAngle3;
    private double redAngle4;
    private double redAngle5;
    private double blueDistance1;
    private double blueDistance2;
    private double blueDistance3;
    private double blueDistance4;
    private double blueDistance5;
    private double blueAngle1;
    private double blueAngle2;
    private double blueAngle3;
    private double blueAngle4;
    private double blueAngle5;
    private double xPos;
    private double angleX;
    private double angleX2;
    private double angleX3;
    private double angleX4;
    private double velocityX;
    

    public ShotMap(RobotState robotState) {
        this.constants = robotState.getConstants().getArmConstants();
        this.robotState = robotState;

        redDistance1 = constants.REDDISTANCE1;
        redDistance2 = constants.REDDISTANCE2;
        redDistance3 = constants.REDDISTANCE3;
        redDistance4 = constants.REDDISTANCE4;
        redDistance5 = constants.REDDISTANCE5;
        redAngle1 = constants.REDANGLE1;//-1
        redAngle2 = constants.REDANGLE2;
        redAngle3 = constants.REDANGLE3;
        redAngle4 = constants.REDANGLE4;
        redAngle5 = constants.REDANGLE5;
        blueDistance1 = constants.BLUEDISTANCE1;
        blueDistance2 = constants.BLUEDISTANCE2;
        blueDistance3 = constants.BLUEDISTANCE3;
        blueDistance4 = constants.BLUEDISTANCE4;
        blueDistance5 = constants.BLUEDISTANCE5;
        blueAngle1 = constants.BLUEANGLE1;//-1
        blueAngle2 = constants.BLUEANGLE2;
        blueAngle3 = constants.BLUEANGLE3;
        blueAngle4 = constants.BLUEANGLE4;
        blueAngle5 = constants.BLUEANGLE5;
    }

    public double getVelocity(){
        velocityX = robotState.getDriveVelocity().getX();
        SmartDashboard.putNumber("VelocityY", velocityX);
        return velocityX;
    }

    public double calculateSlope(double a2, double a1, double d2, double d1, double currentPos){
        angleX = (a2-a1)*(d2-currentPos);
        angleX3 = angleX /(d2-d1);
        angleX4 = a2-angleX3;
        SmartDashboard.putNumber("a1", a1);
        SmartDashboard.putNumber("a2", a2);
        SmartDashboard.putNumber("d1", d1);
        SmartDashboard.putNumber("d2", d2);
        SmartDashboard.putNumber("currentPos", currentPos);

        return angleX4;
    }

    public double calcShotMap(){ 
        if (robotState.getAlliance() == Alliance.Blue) {

            if(xPos<blueDistance1){
                angleX2 = this.calculateSlope(blueAngle1, 152, blueDistance1, 0, xPos);
            }
            else if(xPos>=blueDistance1 && xPos<blueDistance2){
                angleX2 = this.calculateSlope(blueAngle2, blueAngle1, blueDistance2, blueDistance1, xPos);
            }
            else if(xPos>=blueDistance2 && xPos<blueDistance3){
                angleX2 = this.calculateSlope(blueAngle3, blueAngle2, blueDistance3, blueDistance2, xPos);
            }
            else if(xPos>=blueDistance3 && xPos<blueDistance4){
                angleX2 = this.calculateSlope(blueAngle4, blueAngle3, blueDistance4, blueDistance3, xPos);
            }
            else if(xPos>=blueDistance4 && xPos<blueDistance5){
                angleX2 = this.calculateSlope(blueAngle5, blueAngle4, blueDistance5, blueDistance4, xPos);
            } 
            else {
                angleX2 = 95.0;
                SmartDashboard.putNumber("Outside", angleX2);
            }
        } else {
            if(xPos<redDistance1){
                angleX2 = this.calculateSlope(redAngle1, 152, redDistance1, 0, xPos);
            }
            else if(xPos>=redDistance1 && xPos<redDistance2){
                angleX2 = this.calculateSlope(redAngle2, redAngle1, redDistance2, redDistance1, xPos);
            }
            else if(xPos>=redDistance2 && xPos<redDistance3){
                angleX2 = this.calculateSlope(redAngle3, redAngle2, redDistance3, redDistance2, xPos);
            }
            else if(xPos>=redDistance3 && xPos<redDistance4){
                angleX2 = this.calculateSlope(redAngle4, redAngle3, redDistance4, redDistance3, xPos);
            }
            else if(xPos>=redDistance4 && xPos<redDistance5){
                angleX2 = this.calculateSlope(redAngle5, redAngle4, redDistance5, redDistance4, xPos);
            } 
            else {
                angleX2 = 95.0;
                SmartDashboard.putNumber("Outside", angleX2);
            }
        }
        xPos = robotState.getPoseToSpeaker(); //+ 0.1*this.getVelocity();
        SmartDashboard.putNumber("distancetotarget", xPos);

        
        SmartDashboard.putNumber("second", angleX2);
        SmartDashboard.putNumber("VelocityArmMap", this.getVelocity());


        return angleX2;
    }

}
