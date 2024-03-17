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

    boolean ring2First = false;

    Pose2d start = new Pose2d(1.574, 6.109, Rotation2d.fromDegrees(8));
    Pose2d ring1 = new Pose2d(8.2, 7.23, Rotation2d.fromDegrees(0));
    Pose2d midShoot = new Pose2d(4.7, 6.3, Rotation2d.fromDegrees(10));
    Pose2d nextToStage = new Pose2d(5.5, 6.8, Rotation2d.fromDegrees(-5));
    Pose2d ring2 = new Pose2d(8.32, 5.71, Rotation2d.fromDegrees(-14));
    Pose2d almostBetweenRings = new Pose2d(4, 6.30, Rotation2d.fromDegrees(0));
    Pose2d betweenRings = new Pose2d(2.93, 6.30, Rotation2d.fromDegrees(0));
    Pose2d closeShoot = new Pose2d(2.25, 6.3, Rotation2d.fromDegrees(15));
    Pose2d ring3 = new Pose2d(2.87, 6.98, Rotation2d.fromDegrees(28));
    Pose2d backRing4 = new Pose2d(2.1, 5.8, Rotation2d.fromDegrees(0));
    Pose2d ring4 = new Pose2d(2.87, 5.62, Rotation2d.fromDegrees(-5));

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(6, 3.0);

    private void startTraj() {
        // trajectoryConfig.setEndVelocity(0);
        trajectoryConfig = new TrajectoryConfig(6, 3.0);
        if (ring2First) {
            trajectoryGenerator.generate(trajectoryConfig, 
                        List.of(Waypoint.fromHolonomicPose(start, Rotation2d.fromDegrees(0)), 
                                Waypoint.fromHolonomicPose(betweenRings),
                                Waypoint.fromHolonomicPose(nextToStage), 
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
        trajectoryConfig.setEndVelocity(1);
        ring2First = !robotState.getOneNoteFirst();

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

            robotState.setAutonHintXPos(3.6);
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
                            List.of(Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(140)), 
                                    Waypoint.fromHolonomicPose(nextToStage),  
                                    Waypoint.fromHolonomicPose(midShoot)));
                } else {
                    trajectoryGenerator.generate(trajectoryConfig, 
                            List.of(Waypoint.fromHolonomicPose(ring1), 
                                    Waypoint.fromHolonomicPose(midShoot)));
                }
                
                robotState.setAutonHintXPos(calculateArmHint(midShoot));
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
                                    Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(0))));

                } else {
                    trajectoryGenerator.generate(trajectoryConfig,
                            List.of(Waypoint.fromHolonomicPose(midShoot),
                                    Waypoint.fromHolonomicPose(nextToStage), 
                                    Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(-14))));

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
                                    Waypoint.fromHolonomicPose(closeShoot, Rotation2d.fromDegrees(-180))));
                } else {
                    trajectoryGenerator.generate(trajectoryConfig, List.of(
                                    Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(160)),
                                    Waypoint.fromHolonomicPose(nextToStage), 
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

            if (timer.get() > .2) {
                runShooter = true;

            }
            if (timer.get() > .5) {
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
        runIntake = false;
        runShooter = false;
        driving = false;
        ring2First = !robotState.getOneNoteFirst();

        startTraj();

        trajectoryConfig.setEndVelocity(1);
    }
}