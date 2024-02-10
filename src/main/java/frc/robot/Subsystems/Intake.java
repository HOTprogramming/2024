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
     TalonFX intakeTransfer;
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
     static DigitalInput sensorEnter;
     static DigitalInput sensorTransfer;
    TalonFXConfiguration enterConfigs = new TalonFXConfiguration();
    TalonFXConfiguration transferConfigs = new TalonFXConfiguration();

    public Intake(RobotState robotState) { 
        this.robotState = robotState;
         constants = robotState.getConstants().getIntakeConstants();
        sensorEnter = new DigitalInput(constants.ENTER_SENSOR_CHANNEL); 
        sensorTransfer = new DigitalInput(constants.TRANSFER_SENSOR_CHANNEL);
        enterConfigs.Slot0.kP = constants.P0IntakeEnter;
        enterConfigs.Slot0.kI = constants.I0IntakeEnter;
        enterConfigs.Slot0.kD = constants.D0IntakeEnter;
        enterConfigs.Slot0.kV = constants.V0IntakeEnter;
        enterConfigs.Slot1.kP = constants.P1IntakeEnter;
        enterConfigs.Slot1.kI = constants.I1IntakeEnter;
        enterConfigs.Slot1.kD = constants.D1IntakeEnter;
        enterConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        enterConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.EnterPeakForwardTorqueCurrent;
        enterConfigs.TorqueCurrent.PeakReverseTorqueCurrent = constants.EnterPeakReverseTorqueCurrent;
        transferConfigs.Slot0.kP = constants.P0IntakeTransfer;
        transferConfigs.Slot0.kI = constants.I0IntakeTransfer;
        transferConfigs.Slot0.kD = constants.D0IntakeTransfer;
        transferConfigs.Slot0.kV = constants.V0IntakeTransfer;
        transferConfigs.Slot1.kP = constants.P1IntakeTransfer;
        transferConfigs.Slot1.kI = constants.I1IntakeTransfer;
        transferConfigs.Slot1.kD = constants.D1IntakeTransfer;
        transferConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        

        intakeEnter = new TalonFX(constants.INTAKE_ENTER_CAN, "drivetrain");
         intakeTransfer = new TalonFX(constants.INTAKE_TRANSFER_CAN, "drivetrain");

        enterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        StatusCode enterStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            enterStatus = intakeEnter.getConfigurator().apply(enterConfigs);
            if (enterStatus.isOK()) break;
          }
          if(!enterStatus.isOK()) {
            System.out.println("Could not apply configs, error code: " + enterStatus.toString());
          }
        // StatusCode transferStatus = StatusCode.StatusCodeNotInitialized;
        // for (int i = 0; i < 5; ++i) {
        //     transferStatus = intakeTransfer.getConfigurator().apply(enterConfigs);
        //     if (transferStatus.isOK()) break;
        //   }
        //   if(!transferStatus.isOK()) {
        //     System.out.println("Could not apply configs, error code: " + transferStatus.toString());
        //   }
    }


    @Override
    public void updateState() {
        SmartDashboard.putNumber("intake Speed", intakeEnter.getVelocity().getValueAsDouble());
        if ((intakeEnter.getVelocity().getValueAsDouble()) >= (constants.INTAKESPEED - constants.INTAKE_VELOCITY_ERROR) ){
            robotState.setShooterOn(true);
        } else {
            robotState.setShooterOn(false);
        }
    }
    
    @Override
    public void enabled(RobotCommander commander){
         sensorEnter.get();
        sensorTransfer.get();
        SmartDashboard.putNumber("Feeder_Velocity", intakeTransfer.getVelocity().getValue());
        SmartDashboard.putBoolean("enter detection", sensorEnter.get());
     //   SmartDashboard.putBoolean("transfer detection", sensorTransfer.get());
        if (commander.getIntake() || commander.setShoot()) {
            intakeEnter.setControl(Out);
            SmartDashboard.putNumber("Intake RPS", intakeEnter.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber(" Intake set point", constants.INTAKESPEED);
            SmartDashboard.putNumber("Intake error", intakeEnter.getClosedLoopError().getValueAsDouble());
            
            if (sensorEnter.get()/*&& !sensorTransfer.get()*/){
                intakeEnter.setControl(Out);
                if (commander.setShoot()) {
                    intakeTransfer.setControl(m_voltageVelocity.withVelocity(constants.INTAKESPEED));
                } else {
                    intakeTransfer.stopMotor();
                }
                
            }  else { 
               intakeEnter.setControl(m_voltageVelocity.withVelocity(constants.INTAKESPEED));
                intakeTransfer.setControl(m_voltageVelocity.withVelocity(constants.INTAKESPEED));
            }        
            
            

            } else {
                Out.Output = 0;
                intakeEnter.setControl(Out);
                intakeTransfer.stopMotor();
            }

            


            // if (commander.setShoot() || sensorE) {
            //         intakeTransfer.setControl(m_voltageVelocity.withVelocity(constants.INTAKESPEED));
            //     } else {
            //         intakeTransfer.setControl(Out);
            //     }
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