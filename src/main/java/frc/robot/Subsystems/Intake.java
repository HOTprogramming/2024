package frc.robot.Subsystems;


import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.RobotCommander;
import com.ctre.phoenix6.StatusCode;
//import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.DigitalInput;
//import com.ctre.phoenix6.controls.Follower;
//import edu.wpi.first.wpilibj.XboxController;
//import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class Intake implements SubsystemBase {
    ConstantsBase.Intake constants;

   

    RobotState robotState;
    private final DutyCycleOut Out = new DutyCycleOut(0);
    TalonFX intakeEnter;
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
     static DigitalInput sensorFeeder;

    TalonFXConfiguration enterConfigs = new TalonFXConfiguration();

    public Intake(RobotState robotState) { 
        this.robotState = robotState;
         constants = robotState.getConstants().getIntakeConstants();
      //   sensorFeeder = new DigitalInput(constants.FEEDER_SENSOR_CHANNEL);
        enterConfigs.Slot0.kP = constants.P0IntakeEnter;
        enterConfigs.Slot0.kI = constants.I0IntakeEnter;
        enterConfigs.Slot0.kD = constants.D0IntakeEnter;
        enterConfigs.Slot0.kV = constants.V0IntakeEnter;
        enterConfigs.Slot1.kP = constants.P1IntakeEnter;
        enterConfigs.Slot1.kI = constants.I1IntakeEnter;
        enterConfigs.Slot1.kD = constants.D1IntakeEnter;
        enterConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intakeEnter = new TalonFX(constants.INTAKE_ENTER_CAN, "drivetrain");
        enterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        StatusCode enterStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            enterStatus = intakeEnter.getConfigurator().apply(enterConfigs);
            if (enterStatus.isOK()) break;
          }
          if(!enterStatus.isOK()) {
            System.out.println("Could not apply configs, error code: " + enterStatus.toString());
          }
    }


    @Override
    public void updateState() {
        SmartDashboard.putNumber("intake Speed", intakeEnter.getVelocity().getValueAsDouble());
        if ((intakeEnter.getVelocity().getValueAsDouble()) >= (constants.INTAKESPEED - constants.INTAKE_VELOCITY_ERROR) ){
            robotState.setIntakeOn(true);
        } else {
            robotState.setIntakeOn(false);
        }
    }
    
    @Override
    public void enabled(RobotCommander commander){
        // sensorFeeder.get();
        
       // SmartDashboard.putBoolean("Feeder detection", sensorFeeder.get());
        if (commander.getIntake()) {
          intakeEnter.setControl(m_voltageVelocity.withVelocity(constants.INTAKESPEED));

          //  intakeEnter.setControl(Out);
           // SmartDashboard.putNumber("Intake RPS", intakeEnter.getVelocity().getValueAsDouble());
          //  SmartDashboard.putNumber(" Intake set point", constants.INTAKESPEED);
          //  SmartDashboard.putNumber("Intake error", intakeEnter.getClosedLoopError().getValueAsDouble());
          //  if (false){
          //      intakeEnter.setControl(Out);
          //  }  else { 
          //     intakeEnter.setControl(m_voltageVelocity.withVelocity(constants.INTAKESPEED));
          //  }           
            } else {
             Out.Output = 0;
               intakeEnter.setControl(Out);
           }
        }
    

    @Override
    public void disabled() {
        intakeEnter.stopMotor();
    }

    @Override
    public void reset() {
        intakeEnter.stopMotor();
    }


    @Override
    public void init(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }
}