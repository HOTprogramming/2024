package frc.robot.Subsystems;


import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.Subsystems.Arm.ArmCommanded;
import frc.robot.Constants;
import frc.robot.RobotCommander;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
//import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
//import com.ctre.phoenix6.controls.Follower;
//import edu.wpi.first.wpilibj.XboxController;
//import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
    VictorSPX slurperArm;
    VictorSPX slurperSpin;
    CANCoder slurperCancoder;
    private double slurperArmOffset;

    // up = 135.6
    // down = -174

    StatusSignal<Double> sCancoderPosition;
    StatusSignal<Double> sCancoderVelocity;
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
        slurperArm = new VictorSPX(constants.SLURPER_ARM_CAN);
        slurperArm.configFactoryDefault();

        slurperSpin = new VictorSPX(constants.SLURPER_ROLLER_CAN);

        slurperCancoder = new CANCoder(constants.SLURPER_CANCODER_CAN);
        slurperCancoder.configFactoryDefault();
        slurperCancoder.setPositionToAbsolute();
        slurperCancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        slurperCancoder.configMagnetOffset(-278);

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
            if (enterStatus.isOK()) break;
          }
          if(!enterStatus.isOK()) {
            System.out.println("Could not apply configs, error code: " + enterStatus.toString());
          }

        slurperArm.configRemoteFeedbackFilter(slurperCancoder, 0);
        
        slurperArm.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0,
        200);

        //   slurperArm.config

        slurperArm.setSensorPhase(true);
        slurperArm.setInverted(true);

        // slurperCancoder.configSensorDirection(true);

        /* Set the peak and nominal outputs */
        slurperArm.configNominalOutputForward(0, 100);
        slurperArm.configNominalOutputReverse(0, 100);
        slurperArm.configPeakOutputForward(1, 100);
        slurperArm.configPeakOutputReverse(-1, 100);
        
        /* Set Motion Magic gains in slot0 - see documentation */
        slurperArm.selectProfileSlot(0, 0);
        slurperArm.config_kF(0, 0, 100);
        slurperArm.config_kP(0, 0.8, 100);
        slurperArm.config_kI(0, 0, 100);
        slurperArm.config_kD(0, 0, 100);
        //slurperArm.config_IntegralZone(0, this.convertToTicks(0.1));

        /* Set acceleration and vcruise velocity - see documentation */
        slurperArm.configMotionCruiseVelocity(28000, 100);
        slurperArm.configMotionAcceleration(12000, 100);

        // slurperArm.configSelectedFeedbackCoefficient(0.087890625);
        // slurperArm.configSelectedFeedbackCoefficient(0.087890625);
        // slurperArm.configSelectedFeedbackCoefficient((1/4096* 360) );

        this.reset();
        slurperArm.setNeutralMode(NeutralMode.Brake);
    }

    public void intilizeOffset() {
        SmartDashboard.putNumber("Init selectedSensorPosition", slurperArm.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Init slurperCancoder", slurperCancoder.getAbsolutePosition()*4096/360);
        slurperArmOffset = slurperArm.getSelectedSensorPosition(0) - slurperCancoder.getAbsolutePosition()*4096/360;
    }


    @Override
    public void updateState() {
        SmartDashboard.putNumber("intake Speed", intake.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("slurperPos", (slurperArm.getSelectedSensorPosition(0) - slurperArmOffset) * 360/4096);
        SmartDashboard.putNumber("Real Slurper Pos", slurperArm.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("slurperPosCancoder", slurperCancoder.getAbsolutePosition());


        if ((intake.getVelocity().getValueAsDouble()) >= (constants.INTAKESPEED - constants.INTAKE_VELOCITY_ERROR) ){
            robotState.setIntakeOn(true);
        } else {
            robotState.setIntakeOn(false);
        }

        SmartDashboard.putNumber("Slurper Offset", slurperArmOffset);

    }
    
    @Override
    public void enabled(RobotCommander commander){
        SmartDashboard.putNumber("slurper target angle", constants.SLURPER_DOWN_ANGLE);
        // SmartDashboard.putNumber("SlurpDesired", );
      
        // sensorFeeder.get();
        
       // SmartDashboard.putBoolean("Feeder detection", sensorFeeder.get());
        if (commander.getIntake() && !robotState.getBeamBreak()) { // left trigger
            slurperArm.set(ControlMode.MotionMagic, slurperArmOffset -160.0 / 360.0 * 4096.0); 
            intake.setControl(m_voltageVelocity.withVelocity(constants.INTAKESPEED));     
            slurperSpin.set(ControlMode.PercentOutput, .8);
            SmartDashboard.putNumber("SlurpDesiredPos", slurperArmOffset - 160.0 / 360.0 * 4096.0);

            SmartDashboard.putString("Test If running", "running");
        } else if(commander.intakeOut()){
            intake.setControl(m_voltageVelocity.withVelocity(-constants.INTAKESPEED));
            
        }else {
            SmartDashboard.putBoolean("Pulse_check", false); 


            if(commander.armCommanded() != ArmCommanded.handoff && commander.armCommanded() != ArmCommanded.amp && commander.armCommanded() != ArmCommanded.trap){
                slurperSpin.set(ControlMode.PercentOutput, 0); 
            }

            Out.Output = 0;
            intake.setControl(Out);

            slurperArm.set(ControlMode.MotionMagic, slurperArmOffset + 96 / 360.0 * 4096.0); //335.5
            SmartDashboard.putNumber("SlurpDesiredPos", slurperArmOffset + 96 / 360.0 * 4096.0);
        }   
        // } else {
        //     slurperArm.set(ControlMode.PercentOutput, 0);
        //     Out.Output = 0;
        //     intake.setControl(Out);
        //     slurperSpin.set(ControlMode.PercentOutput, 0);
        // }
        SmartDashboard.putNumber("slurperPosCommand", slurperArm.getClosedLoopTarget());
    }
    

    @Override
    public void disabled() {
        
        intake.stopMotor();
    }

    @Override
    public void reset() {
        slurperArm.setSelectedSensorPosition(-slurperCancoder.getAbsolutePosition()*4096/360);
        intake.stopMotor();
        // slurperArm.setSelectedSensorPosition(slurperCancoder.getAbsolutePosition() / 4096);
        this.intilizeOffset();
    }


    @Override
    public void init(RobotCommander commander) {

    }
}