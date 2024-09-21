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


public class AmpRedSpit extends AutonBase {
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

    double spitXValue = 12.49;

    Pose2d start = new Pose2d(14.96, 6.189, Rotation2d.fromDegrees(180));
    Pose2d betweenRings = new Pose2d(13.75, 6.26, Rotation2d.fromDegrees(180));
    Pose2d afterBetweenRings = new Pose2d(12.54, 6.26, Rotation2d.fromDegrees(166));
    Pose2d forkpoint = new Pose2d(10.54, 6.3, Rotation2d.fromDegrees(163));

    Pose2d afterBetweenRingsR2 = new Pose2d(12.54, 6.20, Rotation2d.fromDegrees(158));
    Pose2d forkpointR2 = new Pose2d(10.54, 6.3, Rotation2d.fromDegrees(158));

    Pose2d ring1 = new Pose2d(8.27, 7.7, Rotation2d.fromDegrees(165));
    Pose2d ring1for2first = new Pose2d(8.27, 7.6, Rotation2d.fromDegrees(175));

    Pose2d farShoot = new Pose2d(11.84, 6.3, Rotation2d.fromDegrees(170));
    Pose2d ring2 = new Pose2d(8.35, 5.6, Rotation2d.fromDegrees(194));
    Pose2d aroundStage = new Pose2d(10.75, 6.2, Rotation2d.fromDegrees(180));
    // Pose2d beforeBetweenOtherRings = new Pose2d(4.6, 5.5, Rotation2d.fromDegrees(180));
    // Pose2d betweenOtherRings = new Pose2d(2.9, 4.45, Rotation2d.fromDegrees(180));
    // Pose2d aroundRings = new Pose2d(3.1, 7.5, Rotation2d.fromDegrees(180));
    Pose2d beforeBetweenRings = new Pose2d(12.54, 6.28, Rotation2d.fromDegrees(180));
    Pose2d betweenRingsBack = new Pose2d(13.75, 6.28, Rotation2d.fromDegrees(180));
    Pose2d closeShoot = new Pose2d(14.64, 6.4, Rotation2d.fromDegrees(155));
    // Pose2d backAmpRing = new Pose2d(2.1, 6.5, Rotation2d.fromDegrees(155));
    Pose2d ampRing = new Pose2d(13.73, 6.96, Rotation2d.fromDegrees(155));
    Pose2d backMidRing = new Pose2d(14.24, 5.62, Rotation2d.fromDegrees(180));
    Pose2d midPreback = new Pose2d(13.0, 5.62, Rotation2d.fromDegrees(180));
    // Pose2d backPreload = new Pose2d(1.9, 5.6, Rotation2d.fromDegrees(170));
    Pose2d backStageRing = new Pose2d(14.44, 4.9, Rotation2d.fromDegrees(203));
    Pose2d stageRing = new Pose2d(14.05, 4.6, Rotation2d.fromDegrees(203));

    Pose2d midRingActual = new Pose2d(13.57, 5.56, Rotation2d.fromDegrees(180));
    Pose2d preloadActual = new Pose2d(12.94, 5.56, Rotation2d.fromDegrees(180));


    public AmpRedSpit(RobotState robotState) {
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
                Waypoint.fromHolonomicPose(afterBetweenRingsR2),
                Waypoint.fromHolonomicPose(forkpointR2),
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
            }

            if (robotState.getDrivePose().getX() <= 10.54) {
                runShooter = false;

            } else if (robotState.getDrivePose().getX() <= spitXValue) {
                runShooter = true;
                runIntake = true;

            }

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() || (robotState.getBeamBreak() && robotState.getDrivePose().getX() < 9.54)) {
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
            if (ring1first) {
                robotState.setAutonHintXPos(calculateArmHint(farShoot) + .08);

            } else {
                robotState.setAutonHintXPos(calculateArmHint(farShoot) + .12);

            }
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
                        Waypoint.fromHolonomicPose(ring1for2first)
                    ));
                }

                timer.reset();
                step = Step.tosecring;
            }
        } else if (step == Step.tosecring) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() || (robotState.getBeamBreak() && robotState.getDrivePose().getX() < 9.54)) {
                trajectoryConfig.setEndVelocity(6);
                if (ring1first) {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                        Waypoint.fromHolonomicPose(ring2), 
                        Waypoint.fromHolonomicPose(aroundStage),
                        Waypoint.fromHolonomicPose(beforeBetweenRings, Rotation2d.fromDegrees(0))
                    ));
                } else {
                   trajectoryGenerator.generate(trajectoryConfig, List.of(
                        Waypoint.fromHolonomicPose(ring1),
                        Waypoint.fromHolonomicPose(beforeBetweenRings, Rotation2d.fromDegrees(0))
                    )); 
                }

                step = Step.tocloseshoot;
                timer.reset();
            }
        } else if (step == Step.tocloseshoot) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                
                trajectoryConfig = new TrajectoryConfig(6, 3);
                
                trajectoryConfig.setStartVelocity(6);
                if (ring1first) {
                    trajectoryConfig.setEndVelocity(3.0);

                } else {
                    trajectoryConfig.setEndVelocity(3.5);

                }
                trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(5));
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(beforeBetweenRings, Rotation2d.fromDegrees(0)),
                    Waypoint.fromHolonomicPose(betweenRingsBack, Rotation2d.fromDegrees(0)),
                    Waypoint.fromHolonomicPose(closeShoot, Rotation2d.fromDegrees(90))
                ));

                step = Step.tocloseslow;
                timer.reset();
            }
        } else if (step == Step.tocloseslow) {
            


            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                trajectoryConfig.setEndVelocity(0);
                if (ring1first) {
                    trajectoryConfig.setStartVelocity(3.0);

                } else {
                    trajectoryConfig.setStartVelocity(3.5);

                }
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
                overrideIntake = true;
            }
            if (robotState.getDrivePose().getX() < 14.14) {
                if (ring1first) {

                    robotState.setAutonHintXPos(calculateArmHint(ampRing));
                } else {
                    robotState.setAutonHintXPos(calculateArmHint(ampRing) - .2);

                }
            }
            
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                trajectoryConfig.setStartVelocity(0);
                trajectoryConfig.setEndVelocity(1);
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(ampRing),
                    Waypoint.fromHolonomicPose(backMidRing, Rotation2d.fromDegrees(180))
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
            if (robotState.getDrivePose().getX() < 13.14) {
                runShooter = false;
                overrideIntake = false;
            }

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                trajectoryConfig = new TrajectoryConfig(6, 3);
                trajectoryConfig.setStartVelocity(0);
                trajectoryConfig.setEndVelocity(3);


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
                
                robotState.setAutonHintXPos(calculateArmHint(stageRing) - 0.1);
            }

            if (timer.get() > .7) {
                runShooter = true;
                overrideIntake = true;
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
        ring1first = robotState.getOneNoteFirst();
        robotState.setAutonHintXPos(-1);
        swerveBrake = false;
        overrideIntake = false;
        runShooter = false;
        trajectoryConfig = new TrajectoryConfig(7, 4);
        trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(9));

        trajectoryConfig.setEndVelocity(0);
        trajectoryConfig.setStartVelocity(0);
        startTraj();
        step = Step.start;
    }
}