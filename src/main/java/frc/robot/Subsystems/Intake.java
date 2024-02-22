package frc.robot.Subsystems;


import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.RobotCommander;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
//import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
//import com.ctre.phoenix6.controls.Follower;
//import edu.wpi.first.wpilibj.XboxController;
//import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class Intake implements SubsystemBase {
    ConstantsBase.Intake constants;


    RobotState robotState;
    private final DutyCycleOut Out = new DutyCycleOut(0);
    TalonFX intake;
    TalonFX slurperArm;
    CANcoder cancoder;
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final MotionMagicVoltage slurperMotionMagic = new MotionMagicVoltage(0);
     static DigitalInput sensorFeeder;

    TalonFXConfiguration enterConfigs = new TalonFXConfiguration();
    TalonFXConfiguration slurperArmConfigs = new TalonFXConfiguration();
    TalonFXConfiguration slurperRollerConfigs = new TalonFXConfiguration();

    StatusCode enterStatus = StatusCode.StatusCodeNotInitialized;
    StatusCode slurperArmStatus = StatusCode.StatusCodeNotInitialized;

    public Intake(RobotState robotState) { 
        this.robotState = robotState;
        constants = robotState.getConstants().getIntakeConstants();
        intake = new TalonFX(constants.INTAKE_ENTER_CAN, "drivetrain");
        slurperArm = new TalonFX(constants.SLURPER_ARM_CAN, "rio");
        cancoder = new CANcoder(constants.SLURPER_CANCODER_CAN, "drivetrain");

        MotionMagicConfigs slurperMotionMagic = slurperArmConfigs.MotionMagic;
        slurperMotionMagic.MotionMagicCruiseVelocity = constants.SLURPER_ARM_CRUISE_VELOCITY;
        slurperMotionMagic.MotionMagicAcceleration = constants.SLURPER_ARM_ACCELERATION;
        slurperMotionMagic.MotionMagicJerk = constants.SLURPER_ARM_JERK;

        slurperArmConfigs.Slot0 = constants.SLURPER_ARM_GAINS;
        slurperArmConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        enterConfigs.Slot0.kP = constants.P0IntakeEnter;
        enterConfigs.Slot0.kI = constants.I0IntakeEnter;
        enterConfigs.Slot0.kD = constants.D0IntakeEnter;
        enterConfigs.Slot0.kV = constants.V0IntakeEnter;
        enterConfigs.Slot1.kP = constants.P1IntakeEnter;
        enterConfigs.Slot1.kI = constants.I1IntakeEnter;
        enterConfigs.Slot1.kD = constants.D1IntakeEnter;

        enterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        enterConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        for (int i = 0; i < 5; ++i) {
            enterStatus = intake.getConfigurator().apply(enterConfigs);
            slurperArmStatus = slurperArm.getConfigurator().apply(slurperArmConfigs);
            if (enterStatus.isOK() && slurperArmStatus.isOK()) break;
          }
          if(!enterStatus.isOK() && !slurperArmStatus.isOK()) {
            System.out.println("Could not apply configs, error code: " + enterStatus.toString());
          }
    }


    @Override
    public void updateState() {
        SmartDashboard.putNumber("intake Speed", intake.getVelocity().getValueAsDouble());
        if ((intake.getVelocity().getValueAsDouble()) >= (constants.INTAKESPEED - constants.INTAKE_VELOCITY_ERROR) ){
            robotState.setIntakeOn(true);
        } else {
            robotState.setIntakeOn(false);
        }
    }
    
    @Override
    public void enabled(RobotCommander commander){
        SmartDashboard.putNumber("slurper target angle", constants.SLURPER_DOWN_ANGLE);
        SmartDashboard.putNumber("slurper angle", slurperArm.getPosition().getValueAsDouble() * 360.0);
        // sensorFeeder.get();
        
       // SmartDashboard.putBoolean("Feeder detection", sensorFeeder.get());
        if (commander.getIntake() && !robotState.getFeederStopped()) {
          intake.setControl(m_voltageVelocity.withVelocity(constants.INTAKESPEED));      
        } else {
           Out.Output = 0;
           intake.setControl(Out);
        }

        if (commander.getRunSlurper()) {
            slurperArm.setControl(slurperMotionMagic.withPosition(constants.SLURPER_DOWN_ANGLE/360.0).withSlot(0));
        } else {
            slurperArm.setControl(slurperMotionMagic.withPosition(constants.SLURPER_UP_ANGLE).withSlot(0));
        }
    }
    

    @Override
    public void disabled() {
        
        intake.stopMotor();
    }

    @Override
    public void reset() {
        slurperArm.setPosition(0);
        intake.stopMotor();
    }


    @Override
    public void init(RobotCommander commander) {
        slurperArm.setPosition(0);
    }
}