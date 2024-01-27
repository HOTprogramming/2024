package frc.robot.Autons;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;

public class driveShoot extends AutonBase {
    enum Step {
        driveforward,
        shoot,
        drivebackward,
        shoot2,
        end
    }

    public Step step;

    public driveShoot(RobotState robotState) {
        super(robotState);
    }


    @Override
    public void runAuto() {
        SmartDashboard.putNumber("timer", timer.get());
        switch (step) {
            case driveforward:
                driveSpeed = 0.05;
                if (timer.get() >= 5) {
                    driveSpeed = 0;
                    step = Step.shoot;
                }
                break;
    
            case shoot:
                runShooter = true;
                if (timer.get() >= 8) {
                    runShooter = false;
                    step = Step.drivebackward;
                }
                break;

            case drivebackward:
                driveSpeed = -0.05;
                if (timer.get() >= 13) {
                    driveSpeed = 0;
                    step = Step.shoot2;
                }
                break;

            case shoot2:
                runShooter = true;
                if (timer.get() >= 16) {
                    runShooter = false;
                    step = Step.end;
                }
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
        step = Step.driveforward;

        timer.start();
        timer.reset();
    }
}