// package frc.robot.Autons;
// import java.util.List;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
// import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.RobotState;
// import frc.robot.ConstantsFolder.CompBotConstants.Intake;
// import frc.robot.Subsystems.Arm.ArmCommanded;
// import frc.robot.utils.trajectory.Waypoint;
// public class AmpSideBlue extends AutonBase {
//     enum Step {
//         toring1,
//         toshoot1,
//         toring2,
//         toshoot2,
//         toring3,
//         toshoot3,
//         shoot3,
//         end
//     }
//     public Step step = Step.toring1;
//     public Pose2d shoot1 = new Pose2d(4.25, 6.3, Rotation2d.fromDegrees(9));
//     public Pose2d shoot2 = new Pose2d(4.1, 4.7, Rotation2d.fromDegrees(-7));
//     public Pose2d shoot3 = new Pose2d(2, 6.25, Rotation2d.fromDegrees(0));
//     public Pose2d ring1 = new Pose2d(8, 7, Rotation2d.fromDegrees(0));
//     public Pose2d ring2 = new Pose2d(8, 5.7, Rotation2d.fromDegrees(-30));
//     public Pose2d ring3 = new Pose2d(8, 4.3, Rotation2d.fromDegrees(0));

//     TrajectoryConfig trajectoryConfig = new TrajectoryConfig(6, 3.5);

//     public AmpSideBlue(RobotState robotState) {
//         super(robotState);
//         startPose = new Pose2d(1.4, 6.25, Rotation2d.fromDegrees(0)); //15.15
//         // trajectoryConfig.setEndVelocity(1.5);
//         trajectoryConfig.setEndVelocity(0);
//         initalTraj();
//     }

//     public void initalTraj(){
//         trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(startPose, Rotation2d.fromDegrees(0)),
//                                                         Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(0))));
//     }
//     @Override
//     public void runAuto() {
//         if(step == Step.toring1){
//             driving = true;
//             robotState.setAutonHintXPos(3.76); //3.9 posetospeaker
//             armCommand = ArmCommanded.shotMap; 
//             runIntake = false;
//             swerveBrake = false;
            
//             if(timer.get()<1){
//                 runShooter = false;
//             }
//             else if (timer.get()>=1){
//                 runShooter = true; 
//                 runIntake = true;
//             }
            
//             if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
//                 timer.reset();
//                 step = Step.toshoot1;
//                 trajectoryConfig.setEndVelocity(0);
//                 trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(ring1),
//                                                 Waypoint.fromHolonomicPose(shoot1)));
//                 timer.reset();
//             }
//         } else if(step == Step.toshoot1){ 
//             if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
//                 timer.reset();
//                 step = Step.toshoot2;
//                 trajectoryConfig.setEndVelocity(0);
//                 trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(7));
//                 trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(0)),
//                                                 Waypoint.fromHolonomicPose(ring2),
//                                                 Waypoint.fromHolonomicPose(shoot2, Rotation2d.fromDegrees(100))));
//             }
//         } else if(step == Step.toshoot2){
//             runShooter = false;
//             driving = true;
//             runIntake = true;
//             swerveBrake = false;   
//             robotState.setAutonHintXPos(4);
//             armCommand = ArmCommanded.shotMap;
            
//             if(timer.get() < trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() && timer.get() > (trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()-.5)){
//                 runShooter = true;
//             }

//             if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
//                 timer.reset();
//                 step = Step.toring3;
//                 // trajectoryConfig.setEndVelocity(1.5);
//                 trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(7));
//                 trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(shoot2, Rotation2d.fromDegrees(-30)),
//                                                 Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(0))));
//             }
//         } else if(step == Step.toring3){
//             runShooter = false;
//             driving = true;
//             runIntake = true;
//             swerveBrake = false;   
//             robotState.setAutonHintXPos(5.05);
//             armCommand = ArmCommanded.shotMap; 
            
//             if(timer.get() < trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() && timer.get() > (trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()-.5)){
//                 runShooter = true;
//             }
//             if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
//                 timer.reset();
//                 step = Step.toshoot3;
//                 trajectoryConfig.setEndVelocity(0);
//                 // trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(90)),
//                 //                                 Waypoint.fromHolonomicPose(shoot3, Rotation2d.fromDegrees(180))));
//                 trajectoryGenerator.generate(trajectoryConfig, List.of(Waypoint.fromHolonomicPose(ring3),
//                                 Waypoint.fromHolonomicPose(new Pose2d(7, 6.25, Rotation2d.fromDegrees(0))),
//                                 Waypoint.fromHolonomicPose(new Pose2d(2.5, 6.25, Rotation2d.fromDegrees(20))),
//                                 Waypoint.fromHolonomicPose(new Pose2d(3, 6.75, Rotation2d.fromDegrees(20)))));
//             }
//         } else if(step == Step.toshoot3){
//             if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
//                 step = Step.shoot3;
//             }
//         } else if(step == Step.shoot3){

//         } else {
//             driving = false;
//             runShooter = false;
//             runIntake = false;
//         }
//         SmartDashboard.putString("Step_step", step.toString());
//         if (driving) {
//             swerveBrake = false;
//             holoDriveState = trajectoryGenerator.getDriveTrajectory().sample(timer.get());
//             rotationState = trajectoryGenerator.getHolonomicRotationSequence().sample(timer.get());
//         } else {
//             swerveBrake = true;
//         }
//         visualizePath();
//     }
//     @Override
//     public void reset() {
//         super.reset();
//         swerveBrake = false;
//         step = Step.toring1;
//         trajectoryConfig.setEndVelocity(1);
//         initalTraj();
//     }
// }

package frc.robot.Autons;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
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
        shoot1,
        toring2,
        tocloseshot,
        closeshot,
        toring3,
        shoot3,
        toring4,
        shoot4,
        end
    }
    public Step step = Step.toring1;

    boolean ring2First = true;

    Pose2d start = new Pose2d(1.574, 6.109, Rotation2d.fromDegrees(8));
    Pose2d ring1 = new Pose2d(8.2, 7.13, Rotation2d.fromDegrees(0));
    Pose2d midShoot = new Pose2d(4.7, 6.3, Rotation2d.fromDegrees(14));
    Pose2d ring2 = new Pose2d(8.3, 5.62, Rotation2d.fromDegrees(-14));
    Pose2d almostBetweenRings = new Pose2d(4, 6.109, Rotation2d.fromDegrees(0));
    Pose2d betweenRings = new Pose2d(2.93, 6.109, Rotation2d.fromDegrees(0));
    Pose2d closeShoot = new Pose2d(2.25, 6.3, Rotation2d.fromDegrees(15));
    Pose2d ring3 = new Pose2d(2.97, 7.06, Rotation2d.fromDegrees(24));
    Pose2d backRing4 = new Pose2d(2.3, 5.8, Rotation2d.fromDegrees(0));
    Pose2d ring4 = new Pose2d(3.1, 5.50, Rotation2d.fromDegrees(-5));

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(6, 3.0);

    private void startTraj() {
        trajectoryConfig.setEndVelocity(0);
        trajectoryConfig = new TrajectoryConfig(6, 3.0);
        if (ring2First) {
            trajectoryGenerator.generate(trajectoryConfig, 
                        List.of(Waypoint.fromHolonomicPose(start, Rotation2d.fromDegrees(0)), 
                                Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(-14))));
        } else {
            trajectoryGenerator.generate(trajectoryConfig, 
                        List.of(Waypoint.fromHolonomicPose(start, Rotation2d.fromDegrees(0)), 
                                Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(0))));

        }
}

    public AmpSideBlue(RobotState robotState) {
        super(robotState);
        startPose = start; //15.15
        // trajectoryConfig.setEndVelocity(1.5);
        trajectoryConfig.setEndVelocity(0);
        startTraj();

        driving = false;
        runIntake = false;
        runShooter = false;
        armCommand = ArmCommanded.none;
    }

    @Override
    public void runAuto() {
        if (step == Step.toring1) {
            driving = true;

            armCommand = ArmCommanded.shotMap;

            robotState.setAutonHintXPos(3.5);
            if (timer.get() > 2) {
                runShooter = false;

            } else if (timer.get() > 1) {
                runShooter = true;
                runIntake = true;
            }

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                runShooter = false;
                if (ring2First) {
                    trajectoryGenerator.generate(trajectoryConfig, 
                            List.of(Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(160)), 
                                    Waypoint.fromHolonomicPose(midShoot)));
                } else {
                    trajectoryGenerator.generate(trajectoryConfig, 
                            List.of(Waypoint.fromHolonomicPose(ring1), 
                                    Waypoint.fromHolonomicPose(midShoot)));
                }
                
                robotState.setAutonHintXPos(-1);
                timer.reset();
                step = Step.toshoot1;
            }
        } else if (step == Step.toshoot1) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {

                timer.reset();
                step = Step.shoot1;
            }
        } else if (step == Step.shoot1) {
            driving = false;
            runShooter = true;
            if (timer.get() > .4) {
                runShooter = false;
                driving = true;
                if (ring2First) {
                    trajectoryGenerator.generate(trajectoryConfig,
                            List.of(Waypoint.fromHolonomicPose(midShoot),
                                    Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(0))));

                } else {
                    trajectoryGenerator.generate(trajectoryConfig,
                            List.of(Waypoint.fromHolonomicPose(midShoot),
                                    Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(-14))));

                }
                
                timer.reset();
                step = Step.toring2;
            }

        } else if (step == Step.toring2) {
            if (timer.get() > .3) {
                runShooter = false;
            }

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                if (ring2First) {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                                    Waypoint.fromHolonomicPose(ring1),
                                    Waypoint.fromHolonomicPose(almostBetweenRings),
                                    Waypoint.fromHolonomicPose(betweenRings),
                                    Waypoint.fromHolonomicPose(closeShoot, Rotation2d.fromDegrees(-180))));
                } else {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                                    Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(160)),
                                    Waypoint.fromHolonomicPose(almostBetweenRings),
                                    Waypoint.fromHolonomicPose(betweenRings),
                                    Waypoint.fromHolonomicPose(closeShoot, Rotation2d.fromDegrees(-180))));
                }
                
                timer.reset();
                step = Step.tocloseshot;
            }
        } else if (step == Step.tocloseshot) {

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {

                timer.reset();
                step = Step.closeshot;
            }

        } else if (step == Step.closeshot) {

            driving = false;
            runShooter = true;
            if (timer.get() > .3) {
                runShooter = false;
                driving = true;
                trajectoryConfig = new TrajectoryConfig(3, 2);
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                                    Waypoint.fromHolonomicPose(closeShoot),
                                    Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(24))));

                timer.reset();
                step = Step.toring3;
            }
        } else if (step == Step.toring3) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                                    Waypoint.fromHolonomicPose(ring3),
                                    Waypoint.fromHolonomicPose(backRing4),
                                    Waypoint.fromHolonomicPose(ring4)));
                timer.reset();
                step = Step.shoot3;
            }
        } else if (step == Step.shoot3) {
            runShooter = true;
            driving = false;
            if (timer.get() > .3) {
                driving = true;
                runShooter = false;
                timer.reset();
                step = Step.toring4;
            } 
        } else if (step == Step.toring4) {
           if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                runShooter = true;
                timer.reset();

                step = Step.shoot4;
            }
        } else if (step == Step.shoot4) {
            driving = false;

            if (timer.get() > .5) {
                runShooter = false;
                step = Step.end;
            }
        } else {
            driving = false;
            runIntake = false;
            runShooter = false;
            armCommand = ArmCommanded.none;
        }


        if (driving) {
            swerveBrake = false;
            holoDriveState = trajectoryGenerator.getDriveTrajectory().sample(timer.get());
            rotationState = trajectoryGenerator.getHolonomicRotationSequence().sample(timer.get());
        } else {
            swerveBrake = true;
        }


        SmartDashboard.putString("Step_step", step.toString());
        visualizePath();
    }
    @Override
    public void reset() {
        super.reset();
        swerveBrake = false;
        step = Step.toring1;
        startTraj();

        trajectoryConfig.setEndVelocity(1);
    }
}