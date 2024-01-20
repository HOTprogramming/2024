package frc.robot.Autons;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;


public class driveShoot extends AutonBase {
    enum Step {
        driveForeward,
        driveBackward,
        shoot1,
        shoot2,
        end
    }

    public Step step;

    private double firstPose = 5;
    private double drivePose;

    public driveShoot(RobotState robotState) {
        super(robotState);
    }


    @Override
    public void runAuto() {
        switch (step) {
            case driveForeward: 
                if (robotState.getDrivePose() > 5 ) {
                    driveSpeed = 0;
                    step = Step.shoot1;
                    timer.reset();
                    drivePose = 0;
                }
                else{
                driveSpeed = 0.05;
                drivePose = robotState.getDrivePose();
                
                break;}
        
            case shoot1:
                if (timer.get() > 3 ) {
                    runShooter = false;
                    step = Step.driveBackward;
                    timer.reset();
                }
                else{
                    runShooter = true;
                    break;}
            
            case driveBackward:
                if (robotState.getDrivePose() < -5 ) {
                    driveSpeed = 0;
                    step = Step.shoot2;
                    timer.reset();
                    drivePose = 0;
                }
                else{
                driveSpeed = -0.05;
                drivePose = robotState.getDrivePose();
                break;} 

            case shoot2:
                if (timer.get() > 3 ) {
                    runShooter = false;
                    step = Step.end;
                }
                else{
                    runShooter = true;
                    break; }
            case end:
                runShooter = false;
                break;
        }
    }

    @Override
    public void reset() {
        driveSpeed = 0;
        runShooter = false;
        step = Step.driveForeward;

        timer.start();
        timer.reset();
    }
}