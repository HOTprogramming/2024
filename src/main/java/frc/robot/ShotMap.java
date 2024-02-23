package frc.robot;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;


public class ShotMap {
    RobotState robotState; 
    private double distance1 = 1.16;
    private double distance2 = 2.5;
    private double distance3 = 4;
    private double distance4 = 5.3;
    private double distance5 = 6.5;
    private double angle1 = 151;
    private double angle2 = 134;
    private double angle3 = 123;
    private double angle4 = 119;
    private double angle5 = 118;
    private double xPos = 1.8;
    private double angleX;
    private double angleX2;
    private double angleX3;
    private double angleX4;
    private double velocityX;
    

public ShotMap(RobotState robotState) {
    this.robotState = robotState;
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
    xPos = robotState.getPoseToSpeaker(); //+ 0.1*this.getVelocity();
    SmartDashboard.putNumber("distancetotarget", xPos);

    if(xPos<distance1){
        angleX2 = this.calculateSlope(angle1, 152, distance1, 0, xPos);
    }
    else if(xPos>=distance1 && xPos<distance2){
        angleX2 = this.calculateSlope(angle2, angle1, distance2, distance1, xPos);
    }
    else if(xPos>=distance2 && xPos<distance3){
        angleX2 = this.calculateSlope(angle3, angle2, distance3, distance2, xPos);
    }
    else if(xPos>=distance3 && xPos<distance4){
        angleX2 = this.calculateSlope(angle4, angle3, distance4, distance3, xPos);
    }
    else if(xPos>=distance4 && xPos<distance5){
        angleX2 = this.calculateSlope(angle5, angle4, distance5, distance4, xPos);
    } 
    else {
        angleX2 = 95.0;
    SmartDashboard.putNumber("Outside", angleX2);
    }
    SmartDashboard.putNumber("second", angleX2);
    SmartDashboard.putNumber("VelocityArmMap", this.getVelocity());


return angleX2;
}

}
