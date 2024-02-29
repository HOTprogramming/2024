package frc.robot;

import frc.robot.Autons.AutonBase;
import frc.robot.Subsystems.Arm.ArmCommanded;
import frc.robot.utils.trajectory.RotationSequence;
//import frc.robot.Subsystems.Arm.armDesiredPos;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class AutonCommander implements RobotCommander {
    RobotState robotState;
    AutonBase auto;
    

    public AutonCommander(RobotState robotState) {
        this.robotState = robotState;
        
    }

    public void setAuto(AutonBase selectedAuto) {
        this.auto = selectedAuto;
        this.auto.reset();
    }

    public AutonBase getAuto(){
        return this.auto;
    }

    @Override
    public double[] getDrivePercentCommand() {
        return new double[] {0, 0 ,0};
    }

    public boolean increaseLeftTargetSpeed() {
        return false;
    }

    public boolean decreaseLeftTargetSpeed() {
        return false;
    }

    public boolean increaseRightTargetSpeed() {
        return false;
    }

    public boolean decreaseRightTargetSpeed() {
        return false;
    }

    @Override
    public State getDriveState() {
        return auto.holoDriveState;
    }


    @Override
    public RotationSequence.State getDriveRotationState() {
        return auto.rotationState;
    }

    @Override
    public DriveMode getDriveMode() {
        return DriveMode.stateDrive;
    }

    @Override
    public Pose2d getRefrenceTolerances() {
        return auto.refrenceTolerances;
    }

    @Override
    public Pose2d getOdomretryOverride() {
        return auto.startPose;
    }

    @Override
    public boolean getBrakeCommand() {
        return auto.swerveBrake;
    }

    @Override
    public int getAngleSnapCommand() {
        return -1;
    }

    @Override
    public boolean getPidgeonReset() {
        return false;
    }

    @Override
    public boolean getLockSpeakerCommand() {
        return auto.autoAim;
    }
    public double getRunArm() {
        // return auto.armPos;
        return 0;
    }

    @Override
    public boolean getRunShooter() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public double getTargetDriveSpeed() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getTargetArmSpeed() {
        return 0; 
    }
    
    // @Override
    // public armDesiredPos armPosition() {
    //     // TODO Auto-generated method stub
    //     return armDesiredPos.zero;
    // }

    @Override
    public boolean getLockPoseCommand() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getResetRobotPose() {
        return auto.seedPose;
    }

    @Override
    public boolean getRunFeeder() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntake() {
        // TODO Auto-generated method stub
        return auto.runIntake;
    }
    
    @Override
    public boolean getFeeder() {
        // TODO Auto-generated method stub
        return auto.runIntake;
    }

    @Override
    public boolean setShoot() {
        return auto.runShooter;
    }

    @Override
    public ArmCommanded armCommanded() {
        return auto.armCommand;
    }

    @Override
    public boolean climberUp() {
        return false;
    }

    @Override
    public boolean climberDown() {
        return false;
    }

    @Override
    public double trapArmFineControl() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean climberOverride() {
        // TODO Auto-generated method stub
        return false;
    }

}
