package frc.robot.Autons;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.ConstantsFolder.CompBotConstants.Intake;
import frc.robot.Subsystems.Arm.ArmCommanded;
import frc.robot.utils.trajectory.Waypoint;
public class AmpSideBlue extends AutonBase {
    enum Step {
        toring1,
        toshoot1,
        toring2,
        toshoot2,
        toring3,
        toshoot3,
        shoot3,
        end
    }
    public Step step = Step.toring1;
    public Pose2d shoot1 = new Pose2d(4.8, 6.1, Rotation2d.fromDegrees(3));
    public Pose2d shoot2 = new Pose2d(4.1, 4.8, Rotation2d.fromDegrees(-9));
    public Pose2d shoot3 = new Pose2d(2, 6.25, Rotation2d.fromDegrees(0));
    public Pose2d ring1 = new Pose2d(8.3, 6.2, Rotation2d.fromDegrees(0));
    public Pose2d ring2 = new Pose2d(8.3, 5.7, Rotation2d.fromDegrees(-30));
    public Pose2d ring3 = new Pose2d(8.3, 4.1, Rotation2d.fromDegrees(0));
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(6, 4);
    public AmpSideBlue(RobotState robotState) {
        super(robotState);
        startPose = new Pose2d(1.4, 6.25, Rotation2d.fromDegrees(0)); //15.15
        // trajectoryConfig.setEndVelocity(1.5);
        trajectoryConfig.setEndVelocity(0);
        initalTraj();
    }
    public void initalTraj(){
        trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(startPose, Rotation2d.fromDegrees(0)),
                                                        Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(0))));
    }
    @Override
    public void runAuto() {
        if(step == Step.toring1){
            driving = true;
            robotState.setAutonHintXPos(3.76); //3.9 posetospeaker
            armCommand = ArmCommanded.shotMap; 
            runIntake = false;
            swerveBrake = false;
            
            if(timer.get()<1){
                runShooter = false;
            }
            else if (timer.get()>=1){
                runShooter = true; 
            }
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();
                step = Step.toshoot1;
                trajectoryConfig.setEndVelocity(0);
                trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(ring1),
                                                Waypoint.fromHolonomicPose(shoot1)));
                timer.reset();
            }
        } else if(step == Step.toshoot1){ 
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();
                step = Step.toshoot2;
                trajectoryConfig.setEndVelocity(0);
                trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(7));
                trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(0)),
                                                Waypoint.fromHolonomicPose(ring2),
                                                Waypoint.fromHolonomicPose(shoot2, Rotation2d.fromDegrees(100))));
            }
        } else if(step == Step.toshoot2){
            runShooter = false;
            driving = true;
            runIntake = true;
            swerveBrake = false;   
            robotState.setAutonHintXPos(5.05);
            armCommand = ArmCommanded.shotMap; 
            
            if(timer.get() < trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() && timer.get() > (trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()-1.0)){
                runShooter = true;
            }

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();
                step = Step.toring3;
                // trajectoryConfig.setEndVelocity(1.5);
                trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(7));
                trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(shoot2, Rotation2d.fromDegrees(-30)),
                                                Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(0))));
            }
        } else if(step == Step.toring3){
            runShooter = false;
            driving = true;
            runIntake = true;
            swerveBrake = false;   
            robotState.setAutonHintXPos(5.05);
            armCommand = ArmCommanded.shotMap; 
            
            if(timer.get() < trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() && timer.get() > (trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()-1.0)){
                runShooter = true;
            }
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();
                step = Step.toshoot3;
                trajectoryConfig.setEndVelocity(0);
                // trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(90)),
                //                                 Waypoint.fromHolonomicPose(shoot3, Rotation2d.fromDegrees(180))));
                trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(90)),
                                Waypoint.fromHolonomicPose(new Pose2d(7, 6.25, Rotation2d.fromDegrees(0))),
                                Waypoint.fromHolonomicPose(new Pose2d(2, 6.25, Rotation2d.fromDegrees(20))),
                                Waypoint.fromHolonomicPose(new Pose2d(2.5, 6.75, Rotation2d.fromDegrees(20)))));
            }
        } else if(step == Step.toshoot3){
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                step = Step.shoot3;
            }
        } else if(step == Step.shoot3){
        } else {
            driving = false;
            runShooter = false;
            runIntake = false;
        }
        SmartDashboard.putString("Step_step", step.toString());
        if (driving) {
            swerveBrake = false;
            holoDriveState = trajectoryGenerator.getDriveTrajectory().sample(timer.get());
            rotationState = trajectoryGenerator.getHolonomicRotationSequence().sample(timer.get());
        } else {
            swerveBrake = true;
        }
        visualizePath();
    }
    @Override
    public void reset() {
        super.reset();
        swerveBrake = false;
        step = Step.toring1;
        trajectoryConfig.setEndVelocity(1);
        initalTraj();
    }
}