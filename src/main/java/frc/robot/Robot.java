package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Autons.*;
import frc.robot.Subsystems.Drivetrain;


public class Robot extends TimedRobot {
  private RobotState robotState;
  private TeleopCommander teleopCommander;
  private AutonCommander autonCommander;

  private Drivetrain drivetrain;

  private TestAuton testAuton;
  private WillsSquare willsSquare;

  private final SendableChooser<String> autoSelector = new SendableChooser<>();

  @Override
  public void robotInit() {
    robotState = new RobotState();
    teleopCommander = new TeleopCommander(robotState);
    autonCommander = new AutonCommander(robotState);

    drivetrain = new Drivetrain(robotState);

    testAuton = new TestAuton(robotState);
    willsSquare = new WillsSquare(robotState);

    autoSelector.setDefaultOption("NOT Drive and Shoot", "DriveShoot");
  }

  @Override
  public void robotPeriodic() {
    drivetrain.updateState();

  }

  @Override
  public void autonomousInit() {
    String selectedAuto = autoSelector.getSelected();

    if (selectedAuto == "DriveShoot") {
      autonCommander.setAuto(testAuton);
    } else if (selectedAuto == "WillsSquare") {
      autonCommander.setAuto(willsSquare);
    }

    drivetrain.init(autonCommander);
  }

  @Override
  public void autonomousPeriodic() {
    autonCommander.auto.runAuto();

    drivetrain.enabled(autonCommander);
  }

  @Override
  public void teleopInit() {
    drivetrain.reset();

  }

  @Override
  public void teleopPeriodic() {
    drivetrain.enabled(teleopCommander);

  }

  @Override
  public void disabledInit() {
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
  public void simulationPeriodic() {
    drivetrain.updateSimState(kDefaultPeriod, 12);
  }
}
