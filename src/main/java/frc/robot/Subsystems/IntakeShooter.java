package frc.robot.Subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


public class IntakeShooter implements SubsystemBase {
    RobotState robotState;
    TalonFX shooter1;
    TalonFX shooter2;
    TalonFX floorIntake;
    TalonFX shooterIntake;

    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    

    public IntakeShooter(RobotState robotState) {
        this.robotState = robotState;
        shooter1 = new TalonFX(SHOOTER1_CAN);
        shooter2 = new TalonFX(SHOOTER2_CAN);
        floorIntake = new TalonFX(FLOORINTAKE_CAN);
        shooterIntake = new TalonFX(SHOOTERINTAKE_CAN);
    }

    public void init(){
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Slot0.kP = SHOOTER1KP;
        configs.Slot0.kI = SHOOTER1KI;
        configs.Slot0.kD = SHOOTER1KD; 
        configs.Slot0.kV = SHOOTER1KV; 
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;        
    
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = shooter1.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs, error code: " + status.toString());
        }
    

    TalonFXConfiguration configs2 = new TalonFXConfiguration();

        configs2.Slot0.kP = SHOOTER2KP;
        configs2.Slot0.kI = SHOOTER2KI;
        configs2.Slot0.kD = SHOOTER2KD; 
        configs2.Slot0.kV = SHOOTER2KV; 
        configs2.Voltage.PeakForwardVoltage = 8;
        configs2.Voltage.PeakReverseVoltage = -8;
          
        StatusCode status2 = StatusCode.StatusCodeNotInitialized;
        for (int i2 = 0; i2 < 5; ++i2) {
          status2 = shooter1.getConfigurator().apply(configs2);
          if (status.isOK()) break;
        }
        if(!status2.isOK()) {
          System.out.println("Could not apply configs, error code: " + status2.toString());
        }

    TalonFXConfiguration floorIntakeConfigs = new TalonFXConfiguration();

    floorIntakeConfigs.Slot0.kP = FLOORINTAKEKP;
    floorIntakeConfigs.Slot0.kI = FLOORINTAKEKI;
    floorIntakeConfigs.Slot0.kD = FLOORINTAKEKD; 
    floorIntakeConfigs.Slot0.kV = FLOORINTAKEKV; 
    floorIntakeConfigs.Voltage.PeakForwardVoltage = 8;
    floorIntakeConfigs.Voltage.PeakReverseVoltage = -8;
          
        StatusCode floorIntakeStatus = StatusCode.StatusCodeNotInitialized;
        for (int i3 = 0; i3 < 5; ++i3) {
            floorIntakeStatus = floorIntake.getConfigurator().apply(floorIntakeConfigs);
          if (floorIntakeStatus.isOK()) break;
        }
        if(!floorIntakeStatus.isOK()) {
          System.out.println("Could not apply configs, error code: " + floorIntakeConfigs.toString());
        }

    TalonFXConfiguration shooterIntakeConfigs = new TalonFXConfiguration();

    shooterIntakeConfigs.Slot0.kP = SHOOTERINTAKEKP;
    shooterIntakeConfigs.Slot0.kI = SHOOTERINTAKEKI;
    shooterIntakeConfigs.Slot0.kD = SHOOTERINTAKEKD; 
    shooterIntakeConfigs.Slot0.kV = SHOOTERINTAKEKV; 
    shooterIntakeConfigs.Voltage.PeakForwardVoltage = 8;
    shooterIntakeConfigs.Voltage.PeakReverseVoltage = -8;
            
        StatusCode shooterIntakeStatus = StatusCode.StatusCodeNotInitialized;
        for (int i4 = 0; i4 < 5; ++i4) {
            shooterIntakeStatus = shooterIntake.getConfigurator().apply(shooterIntakeConfigs);
            if (shooterIntakeStatus.isOK()) break;
        }
        if(!shooterIntakeStatus.isOK()) {
            System.out.println("Could not apply configs, error code: " + shooterIntakeConfigs.toString());
        }
    }


    public void shootAtSpeed(int speed) {
        
    }

    @Override
    public void updateState() {
        if (shooter1.getVelocity().getValueAsDouble() >= (SHOOTER1_MAX_SPEED - SHOOTER1_MAX_VELOCITY_ERROR) || shooter2.getVelocity().getValueAsDouble() >= (SHOOTER2_MAX_SPEED - SHOOTER2_MAX_VELOCITY_ERROR)) {
            robotState.setShooterOn(true);
        } 
        else {
            robotState.setShooterOn(false);
        }

        if (floorIntake.getVelocity().getValueAsDouble() >= (FLOORINTAKE_MAX_SPEED - FLOORINTAKE_MAX_VELOCITY_ERROR)) {
            robotState.floorIntakeOn(true);
        }
        else{
            robotState.floorIntakeOn(false);
        }
        if (shooterIntake.getVelocity().getValueAsDouble() >= (SHOOTERINTAKE_MAX_SPEED - SHOOTERINTAKE_MAX_VELOCITY_ERROR)) {
            robotState.shooterIntakeOn(true);
        }
        else{
            robotState.shooterIntakeOn(false);
        }
    }
    
    @Override
    public void enabled(RobotCommander commander) {
        if (commander.getRunShooter()) {
            shooter1.setControl(m_voltageVelocity.withVelocity(SHOOTER1_MAX_SPEED));
            shooter2.setControl(m_voltageVelocity.withVelocity(SHOOTER2_MAX_SPEED));

        } 
        if(commander.getShooterIntake()){
            floorIntake.setControl(m_voltageVelocity.withVelocity(FLOORINTAKE_MAX_SPEED));
            shooterIntake.setControl(m_voltageVelocity.withVelocity(SHOOTERINTAKE_MAX_SPEED));
        }
        else {
            shooter1.set(0);
            shooter2.set(0);
            floorIntake.set(0);
            shooterIntake.set(0);
        }
    }

    @Override
    public void disabled() {
        shooter1.stopMotor();
        shooter2.stopMotor();
        floorIntake.stopMotor();
        shooterIntake.stopMotor();
    }

    @Override
    public void reset() {
        shooter1.stopMotor();
        shooter2.stopMotor();
        floorIntake.stopMotor();
        shooterIntake.stopMotor();
    }
}
