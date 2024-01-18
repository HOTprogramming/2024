package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Autons.driveShoot;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Shooter;


public class Robot extends TimedRobot {
  private RobotState robotState;
  private TeleopCommander teleopCommander;
  private AutonCommander autonCommander;

  private Shooter shooter;
  private Drivetrain drivetrain;

  private driveShoot driveShoot;

  private final SendableChooser<String> autoSelector = new SendableChooser<>();

  @Override
  public void robotInit() {
    robotState = new RobotState();
    teleopCommander = new TeleopCommander(robotState);
    autonCommander = new AutonCommander(robotState);

    shooter = new Shooter(robotState);
    drivetrain = new Drivetrain(robotState);

    driveShoot = new driveShoot(robotState);

    autoSelector.setDefaultOption("Drive and Shoot", "driveShoot");
  }

  @Override
  public void robotPeriodic() {
    shooter.updateState();
    drivetrain.updateState();
  }

  @Override
  public void autonomousInit() {
    String selectedAuto = autoSelector.getSelected();

    if (selectedAuto == "driveShoot") {
      autonCommander.setAuto(driveShoot);
    }
  }

  @Override
  public void autonomousPeriodic() {
    autonCommander.auto.runAuto();
  }

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
