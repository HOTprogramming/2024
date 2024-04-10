package frc.robot.Subsystems;

import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.Subsystems.Arm.ArmCommanded;
import frc.robot.RobotCommander;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder implements SubsystemBase {
    ConstantsBase.Feeder constants;

    private final DutyCycleOut Out = new DutyCycleOut(0);
    TalonFX feeder;
    int timer = 0;
    boolean hasRing = false;
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
     static DigitalInput sensorFeeder;
    TalonFXConfiguration feederConfigs = new TalonFXConfiguration();
    RobotState robotState;

    Timer rumble = new Timer();

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

        feederConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
   
         StatusCode feederStatus = StatusCode.StatusCodeNotInitialized;
         for (int i = 0; i < 5; ++i) {
             feederStatus = feeder.getConfigurator().apply(feederConfigs);
             if (feederStatus.isOK()) break;
        }
           if(!feederStatus.isOK()) {
             System.out.println("Could not apply configs, error code: " + feederStatus.toString());
           }
        rumble.start();

        feeder.getConfigurator().apply(constants.FEEDER_CURRENT_LIMIT);
    }


    @Override
    public void updateState() {
        SmartDashboard.putNumber("intake Speed", feeder.getVelocity().getValueAsDouble());                                              
        if ((feeder.getVelocity().getValueAsDouble()) >= (constants.FEEDERSPEED - constants.FEEDER_VELOCITY_ERROR) ){
            SmartDashboard.putBoolean("Feeder_RSon", true);
            robotState.setFeederOn(true);
        } else {
            SmartDashboard.putBoolean("Feeder_RSon", false);

            robotState.setFeederOn(false);
        }

        robotState.setFeederCurrent(feeder.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("FeederCurrent", feeder.getSupplyCurrent().getValueAsDouble());

        robotState.setBeamBreak(sensorFeeder.get());
        SmartDashboard.putBoolean("beambreak", sensorFeeder.get());

    }
    
    @Override
    public void teleop(RobotCommander commander){
        boolean getFeeder = commander.getFeeder();
        boolean setShoot = commander.setShoot();
        // feeder.setControl(Out);
        SmartDashboard.putBoolean("Feeder getFeeder", commander.getFeeder());
        SmartDashboard.putBoolean("Feeder SetShoot", commander.setShoot());
        SmartDashboard.putBoolean("Feeder_detection", sensorFeeder.get());
        SmartDashboard.putNumber("Feeder RPS", feeder.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(" Feeder set point", constants.FEEDERSPEED);
        SmartDashboard.putNumber("Feeder error", feeder.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("Feeder_position", feeder.getPosition().getValueAsDouble());

        if (!hasRing && sensorFeeder.get()) {
            hasRing = true;
            feeder.setPosition(0);
            rumble.reset();
        } else if (hasRing && !sensorFeeder.get()) {
            hasRing = false;
        }

        if (rumble.get() < 0.55) {
            robotState.setDriverRumble(true);
        } else {
            robotState.setDriverRumble(false);
        }
        commander.driverRumble();

        if (commander.getFeeder() && !commander.setShoot() && commander.armCommanded() != ArmCommanded.handoff) {
            
            if (sensorFeeder.get()){


                if (feeder.getPosition().getValueAsDouble() >= constants.DESIREDENCODERED){ 
                
                    feeder.setControl(Out);

                } else {
                    feeder.setControl(m_voltageVelocity.withVelocity(constants.FEEDERSPEED));
                }
            } else { 
               feeder.setControl(m_voltageVelocity.withVelocity(constants.FEEDERSPEED));
            }           
        } else if (commander.armCommanded() == ArmCommanded.handoff && robotState.getFeederOnAmpTrap()){
            feeder.setControl(m_voltageVelocity.withVelocity(constants.FEEDERSPEED));
        } else if (commander.setShoot()) {
            feeder.setControl(m_voltageVelocity.withVelocity(constants.FEEDERSPEED));
        } else {
            Out.Output = 0;
            feeder.setControl(Out);

        }
    }
        
    @Override
    public void cameraLights() {
        feeder.stopMotor();
        feeder.setNeutralMode(NeutralModeValue.Brake);
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