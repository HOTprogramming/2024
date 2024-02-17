package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Autons.*;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Camera;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Feeder;

public class Robot extends TimedRobot {
  private ConstantsBase constantsBase;
  private RobotState robotState;

  private TeleopCommander teleopCommander;
  private AutonCommander autonCommander;

  private Shooter shooter;
  private Drivetrain drivetrain;
  private Camera camera;
  private Arm arm;
  private Feeder feeder;
  private Intake intake;


  // define subsystem objects

  // define autons (alphabetical)
  private AidenSquare aidenSquare;
  private Blue3Park blue3Park;
  private Blue3Ring blue3Ring;
  private Blue3Under blue3Under;
  private Blue4Ring blue4Ring;
  private Red2Ring red2Ring;
  private Red3Ring red3Ring;
  private Triangle triangle;
  private WillsSquare willsSquare;
  private Red3Left red3Left;
  private Red3Right red3Right; 
  private Close3 close3; 
  private Red4Left red4Left;
  private Blue3Left blue3Left;
  private Blue3Right blue3Right; 
  private PracticeAuto practiceAuto; 

  private StraightLine straightLine;

  // creates autonSelector
  private final SendableChooser<String> autoSelector = new SendableChooser<>();

  @Override
  public void robotInit() {
    constantsBase = new ConstantsBase();
    constantsBase.setAllConstants();
    robotState = new RobotState(constantsBase);
    
    teleopCommander = new TeleopCommander(robotState);
    autonCommander = new AutonCommander(robotState);
    shooter = new Shooter(robotState);
    arm = new Arm(robotState);
    feeder = new Feeder(robotState);
    intake = new Intake(robotState);
    drivetrain = new Drivetrain(robotState);  
    camera = new Camera(robotState);
    intake = new Intake(robotState);

    aidenSquare = new AidenSquare(robotState);
    blue3Park = new Blue3Park(robotState);
    blue3Ring = new Blue3Ring(robotState);
    blue3Under = new Blue3Under(robotState);
    blue4Ring = new Blue4Ring(robotState);
    red2Ring = new Red2Ring(robotState);
    red3Ring = new Red3Ring(robotState);
    triangle = new Triangle(robotState);
    willsSquare = new WillsSquare(robotState);
    red3Left = new Red3Left(robotState);
    red3Right = new Red3Right(robotState);
    close3 = new Close3(robotState);
    red4Left = new Red4Left(robotState);
    blue3Left = new Blue3Left(robotState);
    blue3Right = new Blue3Right( robotState);
    practiceAuto = new PracticeAuto(robotState);

    straightLine = new StraightLine(robotState);

    autoSelector.setDefaultOption("A. Square", "aidenSquare");
    autoSelector.addOption("B3 Park", "blue3Park");
    autoSelector.addOption("B3", "blue3Ring");
    autoSelector.addOption("B3 Under", "blue3Under");
    autoSelector.addOption("B4", "blue4Ring");
    autoSelector.addOption("R2", "red2Ring");
    autoSelector.addOption("Triangle", "triangle");
    autoSelector.addOption("W. Square", "willsSquare");
    arm.armInit();
  }

  @Override
  public void robotPeriodic() {
    camera.updateState();
    drivetrain.updateState(); // drivetrain AFTER camera

    intake.updateState();

    // shooter.updateState();
    arm.updateState();
  }

  @Override
  public void autonomousInit() {
    String selectedAuto = autoSelector.getSelected();


    // // auton selector base
    // if (selectedAuto == "aidenSquare") {
    //   autonCommander.setAuto(aidenSquare);
    // } else if (selectedAuto == "blue3Park") {
    //   autonCommander.setAuto(blue3Park);
    // } else if (selectedAuto == "blue3Ring") {
    //   autonCommander.setAuto(blue3Ring);
    // } else if (selectedAuto == "blue3Under") {
    //   autonCommander.setAuto(blue3Under);
    // } else if (selectedAuto == "blue4Ring") {
    //   autonCommander.setAuto(blue4Ring);
    // } else if (selectedAuto == "red2Ring") {
    //   autonCommander.setAuto(red2Ring);
    // } else if (selectedAuto == "triangle") {
    //   autonCommander.setAuto(triangle);
    // } else if (selectedAuto == "willsSquare") {
    //   autonCommander.setAuto(willsSquare);
    // }

    autonCommander.setAuto(red3Right);


    drivetrain.init(autonCommander);
  }

  @Override
  public void autonomousPeriodic() {
    autonCommander.auto.runAuto();
    shooter.enabled(autonCommander);
    drivetrain.enabled(autonCommander);
    arm.enabled(autonCommander);
    intake.enabled(autonCommander);
    feeder.enabled(autonCommander);
  }

  @Override
  public void teleopInit() {
    robotState.setAlliance(DriverStation.getAlliance().get());
    shooter.reset();
    drivetrain.reset();
    arm.reset();
    intake.reset();
    feeder.reset();
  }

  @Override
  public void teleopPeriodic() {
    shooter.enabled(teleopCommander);
    drivetrain.enabled(teleopCommander);
    arm.enabled(teleopCommander);
    intake.enabled(teleopCommander);
    feeder.enabled(teleopCommander);
  }

  @Override
  public void disabledInit() {
    shooter.disabled();
    drivetrain.disabled();
    arm.disabled();
    feeder.disabled();
    intake.disabled();
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
    arm.simulation();
  }
}
