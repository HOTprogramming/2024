package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private Right4Note right4Note;
  private Center4NoteBlue center4NoteBlue;
  private Right4NoteBlue right4NoteBlue;
  private AndysAuton andysAuton;
  private AmpSideBlue ampSideBlue;
  private AmpSideRed ampSideRed;
  private BlueOppositeAmp blueOppositeAmp;
  private RedOppositeAmp redOppositeAmp;
  private FourRedOppositeAmp fourRedOppositeAmp;
  private FourBlueOppositeAmp fourBlueOppositeAmp;
  private SourceCrazyRed sourceCrazyRed;
  private SourceCrazyBlue sourceCrazyBlue;
  private SourceCenterRingBlue sourceCenterRingBlue;

  // creates autonSelector
  private final SendableChooser<String> autoSelector = new SendableChooser<>();
  private final SendableChooser<String> noteSelector = new SendableChooser<>();

  @Override
  public void robotInit() {
    RobotController.setBrownoutVoltage(5.5);
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
    right4Note = new Right4Note(robotState);
    center4NoteBlue = new Center4NoteBlue(robotState);
    right4NoteBlue = new Right4NoteBlue(robotState);
    andysAuton = new AndysAuton(robotState);
    ampSideBlue = new AmpSideBlue(robotState);
    ampSideRed = new AmpSideRed(robotState);
    blueOppositeAmp = new BlueOppositeAmp(robotState);
    redOppositeAmp = new RedOppositeAmp(robotState);
    fourBlueOppositeAmp = new FourBlueOppositeAmp(robotState);
    fourRedOppositeAmp = new FourRedOppositeAmp(robotState);
    sourceCrazyRed = new SourceCrazyRed(robotState);
    sourceCrazyBlue = new SourceCrazyBlue(robotState);
    sourceCenterRingBlue = new SourceCenterRingBlue(robotState);
    

    newAuto = new NewAuto(robotState);

    noteSelector.setDefaultOption("1 then 2", "12");
    noteSelector.setDefaultOption("2 then 1", "21");

      Shuffleboard.getTab("Competition")
      .add("Note Selector", noteSelector)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withSize(2, 2);

    autoSelector.setDefaultOption("Center", "center");
    autoSelector.addOption("Amp", "amp");
    autoSelector.addOption("Source 3", "source");
    autoSelector.addOption("Source 4", "source4");

      Shuffleboard.getTab("Competition")
      .add("Auto Selector", autoSelector)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withSize(2, 2);



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
    
    String selectedNote = noteSelector.getSelected();

    robotState.setOneNoteFirst(selectedNote.equals("12"));

    // if(selectedAuto.equals("amp") && robotState.getAlliance() == Alliance.Blue){
    //   autonCommander.setAuto(ampSideBlue);
    // } else if(selectedAuto.equals("center") && robotState.getAlliance() == Alliance.Blue){
    //   autonCommander.setAuto(center4NoteBlue);
    // } else if(selectedAuto.equals("amp") && robotState.getAlliance() == Alliance.Red){
    //   autonCommander.setAuto(ampSideRed);
    // } else if(selectedAuto.equals("center") && robotState.getAlliance() == Alliance.Red){
    //   autonCommander.setAuto(center4Note);
    // } else if(selectedAuto.equals("source") && robotState.getAlliance() == Alliance.Red){
    //   autonCommander.setAuto(redOppositeAmp);
    // } else if(selectedAuto.equals("source") && robotState.getAlliance() == Alliance.Blue){
    //   autonCommander.setAuto(blueOppositeAmp);
    // } else if(selectedAuto.equals("source4") && robotState.getAlliance() == Alliance.Blue){
    //   autonCommander.setAuto(fourBlueOppositeAmp);
    // } else if(selectedAuto.equals("source4") && robotState.getAlliance() == Alliance.Red){
    //   autonCommander.setAuto(fourRedOppositeAmp);
    // }

    //autonCommander.setAuto(sourceCrazyRed);
    //autonCommander.setAuto(sourceCrazyBlue);
    autonCommander.setAuto(sourceCenterRingBlue);
    //autonCommander.setAuto(blueOppositeAmp);

    drivetrain.init(autonCommander);
    shooter.reset();
    drivetrain.reset();
    arm.reset();
    intake.reset();
    feeder.reset();
    climber.reset();
    extension.reset();

    drivetrain.autoLimits();
  }

  @Override
  public void autonomousPeriodic() {
    autonCommander.auto.runAuto();
    shooter.teleop(autonCommander);
    drivetrain.teleop(autonCommander);
    arm.teleop(autonCommander);
    intake.teleop(autonCommander);
    feeder.teleop(autonCommander);
    
    lights.cameraLights();
    climber.teleop(autonCommander);   
    extension.teleop(autonCommander);
  }

  @Override
  public void teleopInit() {
    shooter = new Shooter(robotState, 55, 55);
    robotState.setAlliance(DriverStation.getAlliance().get());
    robotState.setAutonHintXPos(-1);
    shooter.reset();
    drivetrain.reset();
    arm.reset();
    intake.reset();
    feeder.reset();
    lights.reset();
    climber.reset();
    climber.init(teleopCommander);
    extension.reset();
    drivetrain.teleLimits();
  }

  @Override
  public void teleopPeriodic() {
    shooter.teleop(teleopCommander);
    drivetrain.teleop(teleopCommander);
    arm.teleop(teleopCommander);
    intake.teleop(teleopCommander);
    feeder.teleop(teleopCommander);
    lights.teleop(teleopCommander);
    climber.teleop(teleopCommander);
    extension.teleop(teleopCommander);
  }

  @Override
  public void disabledInit() {
    shooter.cameraLights();
    drivetrain.cameraLights();
    arm.cameraLights();
    feeder.cameraLights();
    intake.cameraLights();
    lights.cameraLights();
    climber.cameraLights();
    extension.cameraLights();
  }

  @Override
  public void disabledPeriodic() {
    lights.cameraLights();
  }

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
