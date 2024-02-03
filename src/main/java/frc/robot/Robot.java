package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
<<<<<<< Updated upstream
import frc.robot.Autons.*;
import frc.robot.Subsystems.Camera;
import frc.robot.ConstantsFolder.ConstantsBase;
// import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Arm;
=======
import frc.robot.Autons.driveShoot;
import frc.robot.Subsystems.Led;
//import frc.robot.Subsystems.Drivetrain;
//import frc.robot.Subsystems.Intake;
//import frc.robot.Subsystems.Shooter;
>>>>>>> Stashed changes


public class Robot extends TimedRobot {
  private ConstantsBase constantsBase;
  private RobotState robotState;

  private TeleopCommander teleopCommander;
  private AutonCommander autonCommander;

<<<<<<< Updated upstream
  private Shooter shooter;
  private Drivetrain drivetrain;
  private Camera camera;
  private Arm arm;

  // define subsystem objects
=======
 // private Shooter shooter;
 // private Drivetrain drivetrain;
 // private Intake intake;
  private Led led;
  private driveShoot driveShoot;
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
    shooter = new Shooter(robotState);
    arm = new Arm(robotState);
    drivetrain = new Drivetrain(robotState);  
    camera = new Camera(robotState);

    aidenSquare = new AidenSquare(robotState);
    blue3Park = new Blue3Park(robotState);
    blue3Ring = new Blue3Ring(robotState);
    blue3Under = new Blue3Under(robotState);
    blue4Ring = new Blue4Ring(robotState);
    red2Ring = new Red2Ring(robotState);
    red3Ring = new Red3Ring(robotState);
    triangle = new Triangle(robotState);
    willsSquare = new WillsSquare(robotState);

    straightLine = new StraightLine(robotState);

    autoSelector.setDefaultOption("A. Square", "aidenSquare");
    autoSelector.addOption("B3 Park", "blue3Park");
    autoSelector.addOption("B3", "blue3Ring");
    autoSelector.addOption("B3 Under", "blue3Under");
    autoSelector.addOption("B4", "blue4Ring");
    autoSelector.addOption("R2", "red2Ring");
    autoSelector.addOption("Triangle", "triangle");
    autoSelector.addOption("W. Square", "willsSquare");
=======

    //shooter = new Shooter(robotState);
    //drivetrain = new Drivetrain(robotState);
    //intake = new Intake(robotState);
    driveShoot = new driveShoot(robotState);
    led = new Led(robotState);
    autoSelector.setDefaultOption("Drive and Shoot", "driveShoot");
>>>>>>> Stashed changes
  }

  @Override
  public void robotPeriodic() {
<<<<<<< Updated upstream
    camera.updateState();
    drivetrain.updateState(); // drivetrain AFTER camera


    shooter.updateState();
    arm.updateState();
=======
  //  shooter.updateState();
    //drivetrain.updateState();
    //intake.updateState();
    led.updateState();
>>>>>>> Stashed changes
  }

  @Override
  public void autonomousInit() {
    String selectedAuto = autoSelector.getSelected();


    // auton selector base
    if (selectedAuto == "aidenSquare") {
      autonCommander.setAuto(aidenSquare);
    } else if (selectedAuto == "blue3Park") {
      autonCommander.setAuto(blue3Park);
    } else if (selectedAuto == "blue3Ring") {
      autonCommander.setAuto(blue3Ring);
    } else if (selectedAuto == "blue3Under") {
      autonCommander.setAuto(blue3Under);
    } else if (selectedAuto == "blue4Ring") {
      autonCommander.setAuto(blue4Ring);
    } else if (selectedAuto == "red2Ring") {
      autonCommander.setAuto(red2Ring);
    } else if (selectedAuto == "triangle") {
      autonCommander.setAuto(triangle);
    } else if (selectedAuto == "willsSquare") {
      autonCommander.setAuto(willsSquare);
    }

    autonCommander.setAuto(red3Ring);

    drivetrain.init(autonCommander);
  }

  @Override
  public void autonomousPeriodic() {
    autonCommander.auto.runAuto();
<<<<<<< Updated upstream
    // shooter.enabled(autonCommander);
    drivetrain.enabled(autonCommander);
    // arm.enabled(autonCommander);
=======
    //shooter.enabled(autonCommander);
  //  drivetrain.enabled(autonCommander);
   // intake.enabled(autonCommander);
    led.enabled(autonCommander);
>>>>>>> Stashed changes
  }

  @Override
  public void teleopInit() {
<<<<<<< Updated upstream
    shooter.reset();
    drivetrain.reset();
    arm.reset();
    shooter.reset();
=======
   // shooter.reset();
   // drivetrain.reset();
  //  intake.reset();
    led.reset();
>>>>>>> Stashed changes
  }

  @Override
  public void teleopPeriodic() {
<<<<<<< Updated upstream
    shooter.enabled(teleopCommander);
    drivetrain.enabled(teleopCommander);
    arm.enabled(teleopCommander);
    shooter.enabled(teleopCommander);
=======
    //shooter.enabled(teleopCommander);
    //drivetrain.enabled(teleopCommander);
    //intake.enabled(teleopCommander);
    led.enabled(teleopCommander);
>>>>>>> Stashed changes
  }

  @Override
  public void disabledInit() {
<<<<<<< Updated upstream
    shooter.disabled();
    drivetrain.disabled();
    arm.disabled();
    shooter.disabled();
=======
    //shooter.disabled();
    //drivetrain.disabled();
   // intake.disabled();
    led.disabled();
>>>>>>> Stashed changes
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
