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
public class AmpSideRed extends AutonBase {
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

    boolean ring2First = false;


    Pose2d start = new Pose2d(15.164, 6.27, Rotation2d.fromDegrees(165));
    Pose2d ring1 = new Pose2d(8.34, 6.96, Rotation2d.fromDegrees(180));
    Pose2d midShoot = new Pose2d(11.84, 6.3, Rotation2d.fromDegrees(170));
    Pose2d nextToStage = new Pose2d(10.9, 6.2, Rotation2d.fromDegrees(185));
    Pose2d ring2 = new Pose2d(8.244, 5.30, Rotation2d.fromDegrees(210));
    Pose2d almostBetweenRings = new Pose2d(12.54, 6.109, Rotation2d.fromDegrees(180));
    Pose2d betweenRings = new Pose2d(13.61, 6.134, Rotation2d.fromDegrees(180));
    Pose2d closeShoot = new Pose2d(14.29, 6.3, Rotation2d.fromDegrees(165));
    Pose2d ring3 = new Pose2d(13.57, 7.06, Rotation2d.fromDegrees(152));
    Pose2d backRing4 = new Pose2d(14.24, 5.8, Rotation2d.fromDegrees(180));
    Pose2d ring4 = new Pose2d(13.54, 5.50, Rotation2d.fromDegrees(182));

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(6, 3.0);

    private void startTraj() {
        // trajectoryConfig.setEndVelocity(0);
        trajectoryConfig = new TrajectoryConfig(6, 3.0);
        if (ring2First) {
            trajectoryGenerator.generate(trajectoryConfig, 
                        List.of(Waypoint.fromHolonomicPose(start, Rotation2d.fromDegrees(180)),
                                Waypoint.fromHolonomicPose(betweenRings),
                                Waypoint.fromHolonomicPose(nextToStage), 
                                Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(210))));
        } else {
            trajectoryGenerator.generate(trajectoryConfig, 
                        List.of(Waypoint.fromHolonomicPose(start, Rotation2d.fromDegrees(180)), 
                                Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(180))));

        }
}

    public AmpSideRed(RobotState robotState) {
        super(robotState);
        startPose = start; //15.15
        // trajectoryConfig.setEndVelocity(1.5);
        trajectoryConfig.setEndVelocity(1);
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

            robotState.setAutonHintXPos(2.7);
            if (timer.get() > 1.3) {
                runShooter = false;

            } else if (timer.get() > 0.8) {
                runShooter = true;
                runIntake = true;
            }

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                runShooter = false;
                // trajectoryConfig.setEndVelocity(0);
                if (ring2First) {
                    trajectoryGenerator.generate(trajectoryConfig, 
                            List.of(Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(40)),
                                    Waypoint.fromHolonomicPose(nextToStage),  
                                    Waypoint.fromHolonomicPose(midShoot)));
                } else {
                    trajectoryGenerator.generate(trajectoryConfig, 
                            List.of(Waypoint.fromHolonomicPose(ring1), 
                                    Waypoint.fromHolonomicPose(midShoot)));
                }
                
                robotState.setAutonHintXPos(calculateArmHint(midShoot)+.2);
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
            if (timer.get() > .2) {
                runShooter = true;

            }
            if (timer.get() > .5) {
                runShooter = false;
                driving = true;
                trajectoryConfig.setEndVelocity(1);
                robotState.setAutonHintXPos(-1);
                if (ring2First) {
                    trajectoryGenerator.generate(trajectoryConfig,
                            List.of(Waypoint.fromHolonomicPose(midShoot),
                                    Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(180))));

                } else {
                    trajectoryGenerator.generate(trajectoryConfig,
                            List.of(Waypoint.fromHolonomicPose(midShoot),
                                    Waypoint.fromHolonomicPose(nextToStage), 
                                    Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(210))));

                }
                
                timer.reset();
                step = Step.toring2;
            }

        } else if (step == Step.toring2) {

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                // trajectoryConfig.setEndVelocity(0);
                if (ring2First) {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                                    Waypoint.fromHolonomicPose(ring1),
                                    Waypoint.fromHolonomicPose(almostBetweenRings),
                                    Waypoint.fromHolonomicPose(betweenRings),
                                    Waypoint.fromHolonomicPose(closeShoot, Rotation2d.fromDegrees(0))));
                } else {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                                    Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(20)),
                                    Waypoint.fromHolonomicPose(nextToStage), 
                                    Waypoint.fromHolonomicPose(almostBetweenRings),
                                    Waypoint.fromHolonomicPose(betweenRings),
                                    Waypoint.fromHolonomicPose(closeShoot, Rotation2d.fromDegrees(0))));
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

            if (timer.get() > .2) {
                runShooter = true;

            }
            if (timer.get() > .5) {
                runShooter = false;
                driving = true;
                trajectoryConfig = new TrajectoryConfig(3, 2);
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                                    Waypoint.fromHolonomicPose(closeShoot),
                                    Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(156))));

                timer.reset();
                step = Step.toring3;
            }
        } else if (step == Step.toring3) {
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                
                timer.reset();
                step = Step.shoot3;
            }
        } else if (step == Step.shoot3) {
            driving = false;


            if (timer.get() > .2) {
                runShooter = true;

            }
            if (timer.get() > .5) {
                driving = true;
                runShooter = false;

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                                    Waypoint.fromHolonomicPose(ring3),
                                    Waypoint.fromHolonomicPose(backRing4),
                                    Waypoint.fromHolonomicPose(ring4)));
                timer.reset();
                step = Step.toring4;
            } 
        } else if (step == Step.toring4) {
            robotState.setAutonHintXPos(calculateArmHint(ring4)+.2);
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
            robotState.setAutonHintXPos(-1);
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