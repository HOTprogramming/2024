package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Autons.*;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Camera;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.Climber;

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
  private Climber climber;
  private Extension extension;

  // define subsystem objects

  // define autons (alphabetical)

  private Center4Note center4Note;
  private Center4NoteOther center4NoteOther;
  private NewAuto newAuto;

  // creates autonSelector
  private final SendableChooser<String> autoSelector = new SendableChooser<>();

  @Override
  public void robotInit() {
    RobotController.setBrownoutVoltage(5);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    constantsBase = new ConstantsBase();
    constantsBase.setAllConstants();
    robotState = new RobotState(constantsBase);
    
    teleopCommander = new TeleopCommander(robotState);
    autonCommander = new AutonCommander(robotState);
    shooter = new Shooter(robotState, 9, 9);
    arm = new Arm(robotState);
    feeder = new Feeder(robotState);
    intake = new Intake(robotState);
    drivetrain = new Drivetrain(robotState);  
    camera = new Camera(robotState);
    intake = new Intake(robotState);
    lights = new Lights(robotState);
    climber = new Climber(robotState);
    extension = new Extension(robotState);

    center4Note = new Center4Note(robotState);
    center4NoteOther = new Center4NoteOther(robotState);

    newAuto = new NewAuto(robotState);

    autoSelector.setDefaultOption("Right Auto", "right");
    autoSelector.addOption("Center 4 (Left First)", "centerLeft");
    autoSelector.addOption("Center 4 (Right First)", "centerRight");

    arm.armInit();
    extension.extensionInit();
    intake.reset();
  }

  @Override
  public void robotPeriodic() {
    camera.updateState();
    drivetrain.updateState(); // drivetrain AFTER camera

    intake.updateState();
    feeder.updateState();
    shooter.updateState();
    arm.updateState();
    climber.updateState();
    extension.updateState();
  }

  @Override
  public void autonomousInit() {
    shooter = new Shooter(robotState, 60, 60);
    robotState.setAlliance(DriverStation.getAlliance().get());
    String selectedAuto = autoSelector.getSelected();

    // if(selectedAuto.equals("right")){
      // autonCommander.setAuto(newAuto);
    // } else if(selectedAuto.equals("centerLeft")){
      autonCommander.setAuto(center4Note);
    // } else if(selectedAuto.equals("centerRight")){
      // autonCommander.setAuto(center4NoteOther);
    // }

    drivetrain.init(autonCommander);
    shooter.reset();
    drivetrain.reset();
    arm.reset();
    intake.reset();
    feeder.reset();
    climber.reset();
    extension.reset();
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
    climber.enabled(autonCommander);
    extension.enabled(autonCommander);
  }

  @Override
  public void teleopInit() {
    shooter = new Shooter(robotState, 55, 55);
    robotState.setAlliance(DriverStation.getAlliance().get());
    shooter.reset();
    drivetrain.reset();
    arm.reset();
    intake.reset();
    feeder.reset();
    lights.reset();
    climber.reset();
    climber.init(teleopCommander);
    extension.reset();
  }

  @Override
  public void teleopPeriodic() {
    shooter.enabled(teleopCommander);
    drivetrain.enabled(teleopCommander);
    arm.enabled(teleopCommander);
    intake.enabled(teleopCommander);
    feeder.enabled(teleopCommander);
    lights.enabled(teleopCommander);
    climber.enabled(teleopCommander);
    extension.enabled(teleopCommander);
  }

  @Override
  public void disabledInit() {
    shooter.disabled();
    drivetrain.disabled();
    arm.disabled();
    feeder.disabled();
    intake.disabled();
    lights.disabled();
    climber.disabled();
    extension.disabled();
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
