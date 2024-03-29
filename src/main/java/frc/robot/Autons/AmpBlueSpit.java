package frc.robot.Autons;


import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Subsystems.Arm.ArmCommanded;
import frc.robot.utils.trajectory.Waypoint;


public class AmpBlueSpit extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        tofirstring,
        tofirstshoot,
        firstshot,
        tosecring,
        tocloseshoot,
        closeshot,
        toampring,
        ampshoot,
        topreload,
        preshoot,
        tomidring,
        midshoot,
        tostagering,
        stageshoot,
        end
    }

    public Step step;
    // pose 2D here of desired positions
   
    boolean ring1first = true;

    double spitXValue = 3.25;

    Pose2d start = new Pose2d(1.58, 6.189, Rotation2d.fromDegrees(0));
    Pose2d betweenRings = new Pose2d(2.93, 6.20, Rotation2d.fromDegrees(0));
    Pose2d afterBetweenRings = new Pose2d(4, 6.20, Rotation2d.fromDegrees(0));
    Pose2d forkpoint = new Pose2d(6, 6.3, Rotation2d.fromDegrees(0));
    Pose2d ring1 = new Pose2d(8.5, 7.40, Rotation2d.fromDegrees(15));
    Pose2d farShoot = new Pose2d(4.7, 6.3, Rotation2d.fromDegrees(10));
    Pose2d ring2 = new Pose2d(8.5, 5.61, Rotation2d.fromDegrees(-14));
    Pose2d aroundStage = new Pose2d(5.88, 6.3, Rotation2d.fromDegrees(0));
    // Pose2d beforeBetweenOtherRings = new Pose2d(3.6, 4.7, Rotation2d.fromDegrees(0));
    Pose2d betweenOtherRings = new Pose2d(2.9, 4.55, Rotation2d.fromDegrees(0));
    // Pose2d aroundRings = new Pose2d(3.1, 7.5, Rotation2d.fromDegrees(0));
    Pose2d closeShoot = new Pose2d(1.9, 4.7, Rotation2d.fromDegrees(-23));
    Pose2d stageRing = new Pose2d(2.57, 4.31, Rotation2d.fromDegrees(-23));
    Pose2d backMidring = new Pose2d(1.9, 5.2, Rotation2d.fromDegrees(0));
    Pose2d midRing = new Pose2d(2.87, 5.3, Rotation2d.fromDegrees(0));
    Pose2d backPreload = new Pose2d(1.9, 5.6, Rotation2d.fromDegrees(10));
    Pose2d preload = new Pose2d(3.0, 6.1, Rotation2d.fromDegrees(10));
    Pose2d backAmpRing = new Pose2d(2.1, 6.5, Rotation2d.fromDegrees(25));
    Pose2d ampRing = new Pose2d(2.87, 6.80, Rotation2d.fromDegrees(25));
    

    public AmpBlueSpit(RobotState robotState) {
        super(robotState);
        // starting pos code here + any extra inits
        startPose = start;
        seedPose = true;
    }

    private void startTraj() {
        if (ring1first) {
            trajectoryConfig = new TrajectoryConfig(7, 4);
            trajectoryGenerator.generate(trajectoryConfig, List.of(
                Waypoint.fromHolonomicPose(start),
                Waypoint.fromHolonomicPose(betweenRings),
                Waypoint.fromHolonomicPose(afterBetweenRings),
                Waypoint.fromHolonomicPose(forkpoint),
                Waypoint.fromHolonomicPose(ring1)
            ));
        } else {
            trajectoryConfig = new TrajectoryConfig(7, 4);
            trajectoryGenerator.generate(trajectoryConfig, List.of(
                Waypoint.fromHolonomicPose(start),
                Waypoint.fromHolonomicPose(betweenRings),
                Waypoint.fromHolonomicPose(afterBetweenRings),
                Waypoint.fromHolonomicPose(forkpoint),
                Waypoint.fromHolonomicPose(ring2)
            ));
        }
    }


    @Override
    public void runAuto() {
        if (step == Step.start) {
            seedPose = false;
            driving = true;
            armCommand = ArmCommanded.unPackage;
            unPackage = true;
            step = Step.tofirstring;
            timer.reset();

        } else if (step == Step.tofirstring) {
            if (timer.get() > 0.5){
                armCommand = ArmCommanded.spitOut;
                unPackage = false;
                runIntake = true;
            }

            if (robotState.getDrivePose().getX() >= 6) {
                runShooter = false;

            } else if (robotState.getDrivePose().getX() >= spitXValue) {
                runShooter = true;
            }

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                runShooter = false;
                armCommand = ArmCommanded.shotMap;

                if (ring1first) {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                        Waypoint.fromHolonomicPose(ring1), 
                        Waypoint.fromHolonomicPose(farShoot)
                    ));
                } else {
                   trajectoryGenerator.generate(trajectoryConfig, List.of(
                        Waypoint.fromHolonomicPose(ring2), 
                        Waypoint.fromHolonomicPose(farShoot)
                    )); 
                }

                step = Step.tofirstshoot;
                timer.reset();
            }
        } else if (step == Step.tofirstshoot) {

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() - 0.1) {
                step = Step.firstshot;
                driving = false;
                timer.reset();
            }
        } else if (step == Step.firstshot) {
            if (timer.get() > 0.0) {
                runShooter = true;
            }

            if (timer.get() > .3) {
                runShooter = false;
                driving = true;

                if (ring1first) {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                        Waypoint.fromHolonomicPose(farShoot),
                        Waypoint.fromHolonomicPose(ring2)
                    ));
                } else {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                        Waypoint.fromHolonomicPose(farShoot),
                        Waypoint.fromHolonomicPose(ring1)
                    ));
                }

                timer.reset();
                step = Step.tosecring;
            }
        } else if (step == Step.tosecring) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                trajectoryConfig = new TrajectoryConfig(6, 3);
                if (ring1first) {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                        Waypoint.fromHolonomicPose(ring2), 
                        Waypoint.fromHolonomicPose(aroundStage),
                        // Waypoint.fromHolonomicPose(beforeBetweenOtherRings),
                        Waypoint.fromHolonomicPose(betweenOtherRings, Rotation2d.fromDegrees(180)),
                        Waypoint.fromHolonomicPose(closeShoot)
                    ));
                } else {
                   trajectoryGenerator.generate(trajectoryConfig, List.of(
                        Waypoint.fromHolonomicPose(ring1),
                        // Waypoint.fromHolonomicPose(beforeBetweenOtherRings),
                        Waypoint.fromHolonomicPose(betweenOtherRings),                        
                        Waypoint.fromHolonomicPose(closeShoot)
                    )); 
                }

                step = Step.tocloseshoot;
                timer.reset();
            }
        } else if (step == Step.tocloseshoot) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                step = Step.closeshot;
                driving = false;
                timer.reset();
            }

        } else if (step == Step.closeshot) {
            if (timer.get() > 0.0) {
                runShooter = true;
            }

            if (timer.get() > .3) {
                runShooter = false;
                driving = true;
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(closeShoot),
                    Waypoint.fromHolonomicPose(stageRing)
                ));

                timer.reset();
                robotState.setAutonHintXPos(calculateArmHint(midRing));
                step = Step.tostagering;
            }
        } else if (step == Step.tostagering) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                step = Step.stageshoot;
                driving = false;
                timer.reset();
            }
        } else if (step == Step.stageshoot) {
            if (timer.get() > 0.0) {
                runShooter = true;
            }

            if (timer.get() > .3) {
                runShooter = false;
                driving = true;
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(stageRing),
                    Waypoint.fromHolonomicPose(backMidring),
                    Waypoint.fromHolonomicPose(midRing)

                ));
                timer.reset();
                step = Step.tomidring;
            }
        } else if (step == Step.tomidring) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                step = Step.midshoot;
                driving = false;
                timer.reset();
            }
        } else if (step == Step.midshoot) {
            if (timer.get() > 0.0) {
                runShooter = true;
            }

            if (timer.get() > .3) {
                runShooter = false;
                driving = true;
                
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(midRing),
                    Waypoint.fromHolonomicPose(backPreload),
                    Waypoint.fromHolonomicPose(preload)
                ));

                timer.reset();
                step = Step.topreload;
            }
        } else if (step == Step.topreload) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                step = Step.preshoot;
                driving = false;
                timer.reset();
            }
        } else if (step == Step.preshoot) {
            if (timer.get() > 0.0) {
                runShooter = true;
            }

            if (timer.get() > .3) {
                runShooter = false;
                driving = true;
                
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(preload),
                    Waypoint.fromHolonomicPose(backAmpRing),
                    Waypoint.fromHolonomicPose(ampRing)
                ));

                timer.reset();
                step = Step.toampring;
            }
        } else if (step == Step.toampring) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                step = Step.ampshoot;
                driving = false;
                timer.reset();
            }
            
        } else if (step == Step.ampshoot) {
            if (timer.get() > 0.0) {
                runShooter = true;
            }

            if (timer.get() > .3) {
                runShooter = false;
                driving = true;
                
                step = Step.end;
            }
        } else if (step == Step.end) {
            driving = false;
            runIntake = false;
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

        // path visualizer, poses put to dashboard for advantagescope or glass
        visualizePath();
    }

    @Override
    public void reset() {
        super.reset();
        robotState.setAutonHintXPos(-1);
        swerveBrake = false;
        startTraj();
        step = Step.start;
    }
}