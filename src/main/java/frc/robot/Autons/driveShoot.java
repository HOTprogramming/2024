package frc.robot.Autons;

import frc.robot.RobotState;


public class driveShoot extends AutonBase {
    enum Step {
        driveForeward,
        shoot,
        end
    }

    public Step step;

    private double firstPose = 5;
    private double shootEndTime = 10;

    public driveShoot(RobotState robotState) {
        super(robotState);
    }


    @Override
    public void runAuto() {
        switch (step) {
            case driveForeward: 
                if (robotState.getDrivePose() >= firstPose) {
                    driveSpeed = 0;
                    step = Step.shoot;
                }
                driveSpeed = 1;
                break;
        
            case shoot:
                if (timer.get() > shootEndTime) {
                    runShooter = false;
                    step = Step.end;
                }
                runShooter = true;
                break;
                
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