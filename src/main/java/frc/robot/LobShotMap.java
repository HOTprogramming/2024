package frc.robot;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;


public class LobShotMap {

    RobotState robotState; 
    private double distance1 = 0;
    private double distance2 = 4.1;
    private double distance3 = 8.2;
    //3700 left 2700 right
    private double leftSpeed1 = 4050;
    private double rightSpeed1 = 3050;

    private double leftSpeed2 = 3600;
    private double rightSpeed2 = 2600;

    private double leftSpeed3 = 3400;
    private double rightSpeed3 = 2400;

    private double lSpeedY;
    private double lSpeedY2;
    private double lSpeedY3;
    private double lSpeedY4;

    private double rSpeedY;
    private double rSpeedY2;
    private double rSpeedY3;
    private double rSpeedY4;

    private double yPos;

    public LobShotMap(RobotState robotState) {
        this.robotState = robotState;

    }

    public double calculateSlopeLeft(double lS2, double lS1, double lD2, double lD1, double lCurrentPos){
        lSpeedY = (lS2-lS1)*(lD2-lCurrentPos);
        lSpeedY3 = lSpeedY /(lD2-lD1);
        lSpeedY4 = lS2-lSpeedY3;
        return lSpeedY4;
    }

    public double calculateSlopeRight(double rS2, double rS1, double rD2, double rD1, double rCurrentPos){
        rSpeedY = (rS2-rS1)*(rD2-rCurrentPos);
        rSpeedY3 = rSpeedY /(rD2-rD1);
        rSpeedY4 = rS2-rSpeedY3;
        return rSpeedY4;
    }

    public double calcLeftLobShotMap(){ 

             if(yPos>=distance1 && yPos<distance2){
                lSpeedY2 = this.calculateSlopeLeft(leftSpeed2, leftSpeed1, distance2, distance1, yPos);
            }
            else if(yPos>=distance2 && yPos<distance3){
                lSpeedY2 = this.calculateSlopeLeft(leftSpeed3, leftSpeed2, distance3, distance2, yPos);
            }
            else {
                lSpeedY2 = leftSpeed2;
            }
            yPos = robotState.getDrivePose().getY();


        if(robotState.getAlliance() == Alliance.Red){
        return lSpeedY2;// * -50 * robotState.getDriveVelocity().getX();
        }
        else{
            return lSpeedY2;// * 50 * robotState.getDriveVelocity().getX();    
        }
    }

    public double calcRightLobShotMap(){ 
        SmartDashboard.putNumber("RobotVelocityLobShotmap", 50 * robotState.getDriveVelocity().getX());

        if(yPos>=distance1 && yPos<distance2){
           rSpeedY2 = this.calculateSlopeRight(rightSpeed2, rightSpeed1, distance2, distance1, yPos);
       }
       else if(yPos>=distance2 && yPos<distance3){
           rSpeedY2 = this.calculateSlopeRight(rightSpeed3, rightSpeed2, distance3, distance2, yPos);
       }
       else {
           rSpeedY2 = rightSpeed2;
       }
       yPos = robotState.getDrivePose().getY();

       if(robotState.getAlliance() == Alliance.Red){
        return rSpeedY2; //* -50 * robotState.getDriveVelocity().getX();
        }
        else{
            return rSpeedY2; //* 50 * robotState.getDriveVelocity().getX();    
        }
}

}
