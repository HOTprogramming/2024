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
import frc.robot.Subsystems.Lights;

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
  private Lights lights;


  // define subsystem objects

  // define autons (alphabetical)

  private Red3Right red3Right; 
  private NewAuto newAuto;

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
    lights = new Lights(robotState);

    red3Right = new Red3Right(robotState);
    newAuto = new NewAuto(robotState);

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

    shooter.updateState();
    arm.updateState();
    
  }

  @Override
  public void autonomousInit() {
    robotState.setAlliance(DriverStation.getAlliance().get());
    String selectedAuto = autoSelector.getSelected();

    autonCommander.setAuto(newAuto);


    drivetrain.init(autonCommander);
    shooter.reset();
    drivetrain.reset();
    arm.reset();
    intake.reset();
    feeder.reset();
  }

  @Override
  public void autonomousPeriodic() {
    autonCommander.auto.runAuto();
    shooter.enabled(autonCommander);
    drivetrain.enabled(autonCommander);
    arm.enabled(autonCommander);
    intake.enabled(autonCommander);
    feeder.enabled(autonCommander);
    lights.enabled(autonCommander);
  }

  @Override
  public void teleopInit() {
    robotState.setAlliance(DriverStation.getAlliance().get());
    shooter.reset();
    drivetrain.reset();
    arm.reset();
    intake.reset();
    feeder.reset();
    lights.reset();
  }

  @Override
  public void teleopPeriodic() {
    shooter.enabled(teleopCommander);
    drivetrain.enabled(teleopCommander);
    arm.enabled(teleopCommander);
    intake.enabled(teleopCommander);
    feeder.enabled(teleopCommander);
    lights.enabled(teleopCommander);
  }

  @Override
  public void disabledInit() {
    shooter.disabled();
    drivetrain.disabled();
    arm.disabled();
    feeder.disabled();
    intake.disabled();
    lights.disabled();
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
