package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Shooter;


public class Robot extends TimedRobot {
  private RobotState robotState;
  private TeleopCommander teleopCommander;
  // private AutonCommander autonCommander;

  private Shooter shooter;
  private Drivetrain drivetrain;

  @Override
  public void robotInit() {
    robotState = new RobotState();
    teleopCommander = new TeleopCommander(robotState);
    // autonCommander = new AutonCommander(robotState);

    shooter = new Shooter(robotState);
    drivetrain = new Drivetrain(robotState);
  }

  @Override
  public void robotPeriodic() {
    shooter.updateState();
    drivetrain.updateState();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    shooter.reset();
    drivetrain.reset();
  }

  @Override
  public void teleopPeriodic() {
    shooter.enabled(teleopCommander);
    drivetrain.enabled(teleopCommander);
  }

  @Override
  public void disabledInit() {
    shooter.disabled();
    drivetrain.disabled();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
