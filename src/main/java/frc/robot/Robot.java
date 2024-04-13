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
  private SourceFourthRingBlue sourceFourthRingBlue;
  private AmpRedSpit ampRedSpit;
  private AmpBlueSpit ampBlueSpit;

  private SourceCenterRingRed sourceCenterRingRed;
  private SourceFourthRingRed sourceFourthRingRed;

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
    sourceFourthRingBlue = new SourceFourthRingBlue(robotState);
    sourceCenterRingRed = new SourceCenterRingRed(robotState);
    sourceFourthRingRed = new SourceFourthRingRed(robotState);
    ampRedSpit = new AmpRedSpit(robotState);
    ampBlueSpit = new AmpBlueSpit(robotState);


    newAuto = new NewAuto(robotState);

    noteSelector.setDefaultOption("1 then 2", "12");
    noteSelector.setDefaultOption("2 then 1", "21");

      Shuffleboard.getTab("Competition")
      .add("Note Selector", noteSelector)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withSize(2, 2);

    autoSelector.setDefaultOption("Center", "center");
    autoSelector.addOption("Amp", "amp");
    autoSelector.addOption("6 OBJECT AMP", "amp6");
    autoSelector.addOption("Source Center First", "sourceCenter");
    autoSelector.addOption("Source Center Second", "sourceNotCenter");

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
    // } else if(selectedAuto.equals("sourceCenter") && robotState.getAlliance() == Alliance.Red){
    //   autonCommander.setAuto(sourceCenterRingRed);
    // } else if(selectedAuto.equals("sourceCenter") && robotState.getAlliance() == Alliance.Blue){
    //   autonCommander.setAuto(sourceCenterRingBlue);
    // } else if(selectedAuto.equals("sourceNotCenter") && robotState.getAlliance() == Alliance.Red){
    //   autonCommander.setAuto(sourceFourthRingRed);
    // } else if(selectedAuto.equals("sourceNotCenter") && robotState.getAlliance() == Alliance.Blue){
    //   autonCommander.setAuto(sourceFourthRingBlue);
    // } else if(selectedAuto.equals("amp6") && robotState.getAlliance() == Alliance.Blue){
    //   autonCommander.setAuto(ampBlueSpit);
    // } else if(selectedAuto.equals("amp6") && robotState.getAlliance() == Alliance.Red){
    //   autonCommander.setAuto(ampRedSpit);
    // }

    autonCommander.setAuto(sourceCenterRingBlue);


    drivetrain.init(autonCommander);
    shooter.reset();
    drivetrain.reset();
    arm.reset();
    intake.reset();
    feeder.reset();
    climber.reset();
    extension.reset();

    drivetrain.autoLimits();
    intake.autoLimits();
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
    shooter = new Shooter(robotState, 40, 40);
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
    intake.teleLimits();
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
