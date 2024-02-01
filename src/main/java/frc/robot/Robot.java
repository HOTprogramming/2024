package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Autons.*;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Arm;


public class Robot extends TimedRobot {
  private RobotState robotState;
  private TeleopCommander teleopCommander;
  private AutonCommander autonCommander;

  // define subsystem objects
  private Drivetrain drivetrain;
  // private Camera camera;

  private Shooter shooter;

  private Arm arm;

  // define autons
  private TestAuton testAuton;
  private WillsSquare willsSquare;
  private RandomAuto randomAuto;
  private ActualAuton actualAuton;

  // creates autonSelector
  private final SendableChooser<String> autoSelector = new SendableChooser<>();

  @Override
  public void robotInit() {
    robotState = new RobotState();
    teleopCommander = new TeleopCommander(robotState);
    autonCommander = new AutonCommander(robotState);
    shooter = new Shooter(robotState);
    drivetrain = new Drivetrain(robotState);
    // camera = new Camera(robotState);
    shooter = new Shooter(robotState);
    // camera = new Camera(robotState);
    arm = new Arm(robotState);


    testAuton = new TestAuton(robotState);
    willsSquare = new WillsSquare(robotState);
    randomAuto = new RandomAuto(robotState);
    actualAuton = new ActualAuton(robotState);

    autoSelector.setDefaultOption("Testing", "TestAuton");
    autoSelector.addOption("will", "WillsSquare");
  }

  @Override
  public void robotPeriodic() {
    // camera.updateState();
    drivetrain.updateState(); // drivetrain AFTER camera


    shooter.updateState();
    arm.updateState();
  }

  @Override
  public void autonomousInit() {
    String selectedAuto = autoSelector.getSelected();


    if (selectedAuto == "TestAuton") {
      autonCommander.setAuto(testAuton);
    } else if (selectedAuto == "WillsSquare") {
      autonCommander.setAuto(willsSquare);
    }

    autonCommander.setAuto(actualAuton);

    drivetrain.init(autonCommander);
  }

  @Override
  public void autonomousPeriodic() {
    autonCommander.auto.runAuto();
    shooter.enabled(autonCommander);
    drivetrain.enabled(autonCommander);
    arm.enabled(autonCommander);

    drivetrain.enabled(autonCommander);
  }

  @Override
  public void teleopInit() {
    shooter.reset();
    drivetrain.reset();
    arm.reset();
    shooter.reset();
  }

  @Override
  public void teleopPeriodic() {
    shooter.enabled(teleopCommander);
    drivetrain.enabled(teleopCommander);
    arm.enabled(teleopCommander);
    shooter.enabled(teleopCommander);
  }

  @Override
  public void disabledInit() {
    shooter.disabled();
    drivetrain.disabled();
    arm.disabled();
    shooter.disabled();
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
    drivetrain.updateSimState(.02, 12);
  }
}
