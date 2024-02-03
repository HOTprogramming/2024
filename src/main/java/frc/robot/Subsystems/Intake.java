/*package frc.robot.Subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import frc.robot.RobotState;
import frc.robot.RobotCommander;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class Intake implements SubsystemBase {
    RobotState robotState;
    TalonFX intakeEnter;
    TalonFX intakeTransfer;
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    public static DigitalInput sensorEnter = new DigitalInput(ENTER_SENSOR_CHANNEL);
    public static DigitalInput sensorTransfer = new DigitalInput(TRANSFER_SENSOR_CHANNEL);
    TalonFXConfiguration enterConfigs = new TalonFXConfiguration();
    TalonFXConfiguration transferConfigs = new TalonFXConfiguration();

    public Intake(RobotState robotState) {  
        this.robotState = robotState;
        enterConfigs.Slot0.kP = 0.11;
        enterConfigs.Slot0.kI = 0.5;
        enterConfigs.Slot0.kD = 0.0001;
        enterConfigs.Slot0.kV = 0.12;
        enterConfigs.Slot1.kP = 5;
        enterConfigs.Slot1.kI = 0.1;
        enterConfigs.Slot1.kD = 0.001;
        enterConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        enterConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        transferConfigs.Slot0.kP = 0.11;
        transferConfigs.Slot0.kI = 0.5;
        transferConfigs.Slot0.kD = 0.0001;
        transferConfigs.Slot0.kV = 0.12;
        transferConfigs.Slot1.kP = 5;
        transferConfigs.Slot1.kI = 0.1;
        transferConfigs.Slot1.kD = 0.001;
        transferConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        transferConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        intakeEnter = new TalonFX(INTAKE_ENTER_CAN);
        intakeTransfer = new TalonFX(INTAKE_TRANSFER_CAN);

        StatusCode enterStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            enterStatus = intakeEnter.getConfigurator().apply(enterConfigs);
            if (enterStatus.isOK()) break;
          }
          if(!enterStatus.isOK()) {
            System.out.println("Could not apply configs, error code: " + enterStatus.toString());
          }
        StatusCode transferStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            transferStatus = intakeTransfer.getConfigurator().apply(enterConfigs);
            if (transferStatus.isOK()) break;
          }
          if(!transferStatus.isOK()) {
            System.out.println("Could not apply configs, error code: " + transferStatus.toString());
          }
    }


    @Override
    public void updateState() {
        SmartDashboard.putNumber("intake Speed", intakeEnter.getVelocity().getValueAsDouble());
        if ((intakeEnter.getVelocity().getValueAsDouble()) >= (INTAKESPEED - INTAKE_VELOCITY_ERROR) ){
            robotState.setShooterOn(true);
        } else {
            robotState.setShooterOn(false);
        }
    }
    
    @Override
    public void enabled(RobotCommander commander){
        sensorEnter.get();
        sensorTransfer.get();
        SmartDashboard.putBoolean("enter detection", sensorEnter.get());
        SmartDashboard.putBoolean("transfer detection", sensorTransfer.get());
        if (commander.getIntake()){
            if (!sensorEnter.get() && !sensorTransfer.get()){
                intakeEnter.setControl(m_voltageVelocity.withVelocity(INTAKESPEED));
                intakeTransfer.setControl(m_voltageVelocity.withVelocity(INTAKESPEED));
            } else if (sensorEnter.get() && !sensorTransfer.get()){
                intakeEnter.setControl(m_voltageVelocity.withVelocity(INTAKESPEED));
                intakeTransfer.setControl(m_voltageVelocity.withVelocity(INTAKESPEED));                
            } else {
                intakeEnter.set(0);
                intakeTransfer.set(0);
            }
        } else {
            intakeEnter.set(0);
            intakeTransfer.set(0);
        }
    }

    @Override
    // public void enabled(RobotCommander commander) {
    //     if (commander.getIntake()) {
    //         intakeMotor.setControl(m_voltageVelocity.withVelocity(INTAKESPEED));
    //     } else {
    //         intakeEnter.set(0);
    //     }
    // }

    // @Override
    public void disabled() {
        intakeEnter.stopMotor();
    }

    @Override
    public void reset() {
        intakeEnter.stopMotor();
    }
}*/