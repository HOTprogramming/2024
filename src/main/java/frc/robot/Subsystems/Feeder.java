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

public class Feeder implements SubsystemBase {
    ConstantsBase.Feeder constants;

    private final DutyCycleOut Out = new DutyCycleOut(0);
    TalonFX feeder;
    int timer = 0;
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
     static DigitalInput sensorFeeder;
    TalonFXConfiguration feederConfigs = new TalonFXConfiguration();
RobotState robotState;
    public Feeder(RobotState robotState) { 
        this.robotState = robotState;
        constants = robotState.getConstants().getFeederConstants();
        sensorFeeder = new DigitalInput(constants.FEEDER_SENSOR_CHANNEL);
        feederConfigs.Slot0.kP = constants.P0IntakeFeeder;
        feederConfigs.Slot0.kI = constants.I0IntakeFeeder;
        feederConfigs.Slot0.kD = constants.D0IntakeFeeder;
        feederConfigs.Slot0.kV = constants.V0IntakeFeeder;
        feederConfigs.Slot1.kP = constants.P1IntakeFeeder;
        feederConfigs.Slot1.kI = constants.I1IntakeFeeder;
        feederConfigs.Slot1.kD = constants.D1IntakeFeeder;
        feederConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        feeder = new TalonFX(constants.FEEDER_CAN, "drivetrain");

        feederConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
   
         StatusCode feederStatus = StatusCode.StatusCodeNotInitialized;
         for (int i = 0; i < 5; ++i) {
             feederStatus = feeder.getConfigurator().apply(feederConfigs);
             if (feederStatus.isOK()) break;
       }
           if(!feederStatus.isOK()) {
             System.out.println("Could not apply configs, error code: " + feederStatus.toString());
           }
    }


    @Override
    public void updateState() {
        SmartDashboard.putNumber("intake Speed", feeder.getVelocity().getValueAsDouble());
        if ((feeder.getVelocity().getValueAsDouble()) >= (constants.FEEDERSPEED - constants.FEEDER_VELOCITY_ERROR) ){
            robotState.setFeederOn(true);
        } else {
            robotState.setFeederOn(false);
        }
    }
    
    @Override
    public void enabled(RobotCommander commander){
       if (sensorFeeder.get()){
            robotState.setFeederStopped(true);
            timer += 1;
        } else {
            robotState.setFeederStopped(false);
            timer = 0;
        }
           SmartDashboard.putBoolean("Feeder detection", sensorFeeder.get());
        if (commander.getFeeder() || commander.setShoot()) {
            feeder.setControl(Out);
            SmartDashboard.putNumber("Feeder RPS", feeder.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber(" Feeder set point", constants.FEEDERSPEED);
            SmartDashboard.putNumber("Feeder error", feeder.getClosedLoopError().getValueAsDouble());
            if (sensorFeeder.get()){
                if (timer >= constants.DESIREDTIMER){
                
                    if (commander.setShoot()) {
                        feeder.setControl(m_voltageVelocity.withVelocity(constants.FEEDERSPEED));
                    } else {
                        feeder.setControl(Out);
                    }

                } else {
                    feeder.setControl(m_voltageVelocity.withVelocity(constants.FEEDERSPEED));
                }
            } else { 
               feeder.setControl(m_voltageVelocity.withVelocity(constants.FEEDERSPEED));
            }           
            } else {
                Out.Output = 0;
                feeder.setControl(Out);
            }
        }
        
    @Override
    public void disabled() {
        feeder.stopMotor();
    }

    @Override
    public void reset() {
        feeder.stopMotor();
    }

    @Override
    public void init(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }
}