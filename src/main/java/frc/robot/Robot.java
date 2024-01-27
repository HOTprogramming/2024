package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Autons.driveShoot;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.IntakeShooter;


public class Robot extends TimedRobot {
  private RobotState robotState;
  private TeleopCommander teleopCommander;
  private AutonCommander autonCommander;

  private IntakeShooter intakeShooter;
  private Drivetrain drivetrain;

  private driveShoot driveShoot;

  private Arm arm;

  private final SendableChooser<String> autoSelector = new SendableChooser<>();

  @Override
  public void robotInit() {
    robotState = new RobotState();
    teleopCommander = new TeleopCommander(robotState);
    autonCommander = new AutonCommander(robotState);

    intakeShooter = new IntakeShooter(robotState);
    drivetrain = new Drivetrain(robotState);

    driveShoot = new driveShoot(robotState);
    arm = new Arm(robotState);

    autoSelector.setDefaultOption("Drive and Shoot", "driveShoot");
    
    intakeShooter.init();

    arm.armInit();

  }

  @Override
  public void robotPeriodic() {
    intakeShooter.updateState();
    drivetrain.updateState();
    arm.updateState();
  }

  @Override
  public void autonomousInit() {
    String selectedAuto = autoSelector.getSelected();

    if (selectedAuto == "driveShoot") {
      autonCommander.setAuto(driveShoot);
    }
    autonCommander.auto.reset();
  }

  @Override
  public void autonomousPeriodic() {
    autonCommander.auto.runAuto();
    intakeShooter.enabled(autonCommander);
    drivetrain.enabled(autonCommander);
    arm.enabled(autonCommander);

  }

  @Override
  public void teleopInit() {
    intakeShooter.reset();
    drivetrain.reset();
    arm.reset();
  }

  @Override
  public void teleopPeriodic() {
    intakeShooter.enabled(teleopCommander);
    drivetrain.enabled(teleopCommander);
    arm.enabled(teleopCommander);
  }

  @Override
  public void disabledInit() {
    intakeShooter.disabled();
    drivetrain.disabled();
    arm.disabled();
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
