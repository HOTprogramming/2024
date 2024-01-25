package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        return new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }


    @Override
    public Pose2d getOdomretryOverride() {
        // will be camera stuff
        return null;
    }


    @Override
    public boolean getBrakeCommand() {
        return driver.getAButton();
    }

    @Override
    public int getAngleSnapCommand() {
        int theta = driver.getPOV();
        if (theta == 90 || theta == 270) {
            theta += 180;
        }
        SmartDashboard.putNumber("POV", theta);
        return theta;
    }

    @Override
    public boolean getPidgeonReset() {
        return driver.getStartButton();              
    }  
}
