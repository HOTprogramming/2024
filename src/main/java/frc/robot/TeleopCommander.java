package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.trajectory.RotationSequence;

public class TeleopCommander implements RobotCommander {
    private static XboxController driver;
    private static XboxController operator;


    RobotState robotState;


    public TeleopCommander(RobotState robotState) {
        this.robotState = robotState;

        driver = new XboxController(0);
        operator = new XboxController(1);

    }

   @Override
    public double[] getDrivePercentCommand() {
        return new double[] {-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX()};
    }

    @Override
    public State getDriveState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDriveState'");
    }


    @Override
    public RotationSequence.State getDriveRotationState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDriveRotationState'");
    }

    @Override
    public DriveMode getDriveMode() {
        return DriveMode.percent;
    }


    @Override
    public Pose2d getRefrenceTolerances() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRefrenceTolerances'");
    }


    @Override
    public Pose2d getOdomretryOverride() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getOdomretryOverride'");
    }


    @Override
    public boolean getBrakeCommand() {
        return driver.getAButton();
    }  
}
