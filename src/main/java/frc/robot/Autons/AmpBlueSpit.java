package frc.robot.Autons;


import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;

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
        tocloseslow,
        closeshot,
        toampring,
        ampshoot,
        topreload,
        preshoot,
        tomidring,
        tomidringslow,
        midshoot,
        tostagering,
        stageshoot,
        end
    }

    public Step step;
    // pose 2D here of desired positions
   
    boolean ring1first = true;

    double spitXValue = 4.18; // 3.92 ILLEAGAL

    Pose2d start = new Pose2d(1.58, 6.189, Rotation2d.fromDegrees(0));
    Pose2d betweenRings = new Pose2d(2.93, 6.20, Rotation2d.fromDegrees(0));
    Pose2d afterBetweenRings = new Pose2d(4, 6.20, Rotation2d.fromDegrees(20)); // 25 ILL
    Pose2d forkpoint = new Pose2d(6, 6.3, Rotation2d.fromDegrees(20)); // 25 ILL
    Pose2d ring1 = new Pose2d(8.15, 7.80, Rotation2d.fromDegrees(15));
    Pose2d farShoot = new Pose2d(4.7, 6.7, Rotation2d.fromDegrees(10));
    Pose2d ring2 = new Pose2d(8.26, 5.81, Rotation2d.fromDegrees(-14));
    Pose2d aroundStage = new Pose2d(5.5, 6.15, Rotation2d.fromDegrees(0));
    // Pose2d beforeBetweenOtherRings = new Pose2d(4.6, 5.5, Rotation2d.fromDegrees(0));
    // Pose2d betweenOtherRings = new Pose2d(2.9, 4.45, Rotation2d.fromDegrees(0));
    // Pose2d aroundRings = new Pose2d(3.1, 7.5, Rotation2d.fromDegrees(0));
    Pose2d beforeBetweenRings = new Pose2d(4, 6.27, Rotation2d.fromDegrees(0));
    Pose2d betweenRingsBack = new Pose2d(2.93, 6.27, Rotation2d.fromDegrees(0));
    Pose2d closeShoot = new Pose2d(1.9, 6.4, Rotation2d.fromDegrees(25));
    // Pose2d backAmpRing = new Pose2d(2.1, 6.5, Rotation2d.fromDegrees(25));
    Pose2d ampRing = new Pose2d(2.82, 6.97, Rotation2d.fromDegrees(25));
    Pose2d backMidRing = new Pose2d(2.3, 5.4, Rotation2d.fromDegrees(0));
    Pose2d midPreback = new Pose2d(3.5, 5.55, Rotation2d.fromDegrees(0));
    // Pose2d backPreload = new Pose2d(1.9, 5.6, Rotation2d.fromDegrees(10));
    Pose2d backStageRing = new Pose2d(2.2, 4.61, Rotation2d.fromDegrees(-23));
    Pose2d stageRing = new Pose2d(2.56, 4.30, Rotation2d.fromDegrees(-23));

    Pose2d midRingActual = new Pose2d(2.9, 5.55, Rotation2d.fromDegrees(0));
    Pose2d preloadActual = new Pose2d(3.6, 5.56, Rotation2d.fromDegrees(0));


    public AmpBlueSpit(RobotState robotState) {
        super(robotState);
        // starting pos code here + any extra inits
        startPose = start;
        ring1first = robotState.getOneNoteFirst();

        seedPose = true;
    }

    private void startTraj() {
        if (ring1first) {
            trajectoryConfig = new TrajectoryConfig(7, 4);
            trajectoryConfig.setEndVelocity(1);
            trajectoryGenerator.generate(trajectoryConfig, List.of(
                Waypoint.fromHolonomicPose(start),
                Waypoint.fromHolonomicPose(betweenRings),
                Waypoint.fromHolonomicPose(afterBetweenRings),
                Waypoint.fromHolonomicPose(forkpoint),
                Waypoint.fromHolonomicPose(ring1)
            ));
        } else {
            trajectoryConfig = new TrajectoryConfig(7, 4);
            trajectoryConfig.setEndVelocity(1);
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
                armCommand = ArmCommanded.mayaspit;
                unPackage = false;
                runIntake = true;
            }

            if (robotState.getDrivePose().getX() >= 6) {
                runShooter = false;

            } else if (robotState.getDrivePose().getX() >= spitXValue) {
                runShooter = true;
            }

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() || (robotState.getBeamBreak() && robotState.getDrivePose().getX() > 7)) {
                runShooter = false;
                armCommand = ArmCommanded.shotMap;
                trajectoryConfig.setEndVelocity(0);

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
            robotState.setAutonHintXPos(calculateArmHint(farShoot));
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() - 0.1) {
                step = Step.firstshot;
                driving = false;
                timer.reset();
            }
        } else if (step == Step.firstshot) {
            if (timer.get() > 0.0) {
                runShooter = true;
            }

            if (timer.get() > .2 || !robotState.getBeamBreak()) {
                runShooter = false;
                driving = true;
                trajectoryConfig.setEndVelocity(1);

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
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() || (robotState.getBeamBreak() && robotState.getDrivePose().getX() > 7)) {
                trajectoryConfig.setEndVelocity(6);
                if (ring1first) {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                        Waypoint.fromHolonomicPose(ring2), 
                        Waypoint.fromHolonomicPose(aroundStage),
                        Waypoint.fromHolonomicPose(beforeBetweenRings)
                    ));
                } else {
                   trajectoryGenerator.generate(trajectoryConfig, List.of(
                        Waypoint.fromHolonomicPose(ring1),
                        Waypoint.fromHolonomicPose(beforeBetweenRings)
                    )); 
                }

                step = Step.tocloseshoot;
                timer.reset();
            }
        } else if (step == Step.tocloseshoot) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                
                trajectoryConfig = new TrajectoryConfig(6, 3);
                
                trajectoryConfig.setStartVelocity(6);
                trajectoryConfig.setEndVelocity(2);
                trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(5));
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(beforeBetweenRings),
                    Waypoint.fromHolonomicPose(betweenRingsBack),
                    Waypoint.fromHolonomicPose(closeShoot, Rotation2d.fromDegrees(90))
                ));

                step = Step.tocloseslow;
                timer.reset();
            }
        } else if (step == Step.tocloseslow) {
            


            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                trajectoryConfig.setEndVelocity(0);
                trajectoryConfig.setStartVelocity(2);
                trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(9));

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(closeShoot, Rotation2d.fromDegrees(90)),
                    Waypoint.fromHolonomicPose(ampRing)
                    ));
                robotState.setAutonHintXPos(calculateArmHint(closeShoot) + .5);

                step = Step.toampring;
                timer.reset();
            }
        } else if (step == Step.toampring) {
            if (timer.get() > .4) {
                runShooter = true;
            }
            if (robotState.getDrivePose().getX() > 2.4) {
                robotState.setAutonHintXPos(calculateArmHint(ampRing));
            }
            
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                trajectoryConfig.setStartVelocity(0);
                trajectoryConfig.setEndVelocity(1);
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(ampRing),
                    Waypoint.fromHolonomicPose(backMidRing, Rotation2d.fromDegrees(0))
                ));

                step = Step.tomidring;
                timer.reset();
            }
        } else if (step == Step.tomidring) {
            robotState.setAutonHintXPos(calculateArmHint(midRingActual) + .3);

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                trajectoryConfig = new TrajectoryConfig(2.85, 3);
                trajectoryConfig.setStartVelocity(1);
                trajectoryConfig.setEndVelocity(0);
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(backMidRing),
                    Waypoint.fromHolonomicPose(midPreback)                    
                ));
                step = Step.tomidringslow;
                timer.reset();
            }
        } else if (step == Step.tomidringslow) {
            // if (robotState.getDrivePose().getX() > 3.4) {
            //     robotState.setAutonHintXPos(calculateArmHint(preloadActual));
            // }
            if (robotState.getDrivePose().getX() > 3.4) {
                runShooter = false;

            }

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                trajectoryConfig = new TrajectoryConfig(6, 3);
                trajectoryConfig.setStartVelocity(0);

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(midPreback),
                    Waypoint.fromHolonomicPose(backStageRing),
                    Waypoint.fromHolonomicPose(stageRing)                    
                ));

                
                step = Step.tostagering;
                timer.reset();
            }
        } else if (step == Step.tostagering) {
            if (timer.get() > .4) {
                
                robotState.setAutonHintXPos(calculateArmHint(stageRing));
            }

            if (timer.get() > .7) {
                runShooter = true;
            }

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() + 2) {
                step = Step.end;
                timer.reset();
            }
        } else if (step == Step.end) {
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
        

        SmartDashboard.putNumber("Step_traj_SV", trajectoryConfig.getStartVelocity());
        SmartDashboard.putNumber("Step_traj_EV", trajectoryConfig.getEndVelocity());
        SmartDashboard.putNumber("Step_traj_MV", trajectoryConfig.getMaxVelocity());

        SmartDashboard.putString("Step_step", step.toString());

        // path visualizer, poses put to dashboard for advantagescope or glass
        visualizePath();
    }

    @Override
    public void reset() {
        super.reset();
        robotState.setAutonHintXPos(-1);
        ring1first = robotState.getOneNoteFirst();
        swerveBrake = false;
        trajectoryConfig = new TrajectoryConfig(7, 4);
        trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(9));

        trajectoryConfig.setEndVelocity(0);
        trajectoryConfig.setStartVelocity(0);
        startTraj();
        step = Step.start;
    }
}