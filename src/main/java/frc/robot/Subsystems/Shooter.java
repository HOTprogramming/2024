package frc.robot.Subsystems;

import frc.robot.RobotCommander;
import frc.robot.RobotState;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.Subsystems.Arm.ArmCommanded;

public class Shooter implements SubsystemBase {
    RobotState robotState;
    TalonFX leftFlywheel;
    TalonFX rightFlywheel;

    VelocityVoltage leftVoltageVelocity;
    VelocityVoltage rightVoltageVelocity;
    VelocityTorqueCurrentFOC leftTorqueCurrentFOC;
    VelocityTorqueCurrentFOC rightTorqueCurrentFOC;
    boolean isShooting = false;
    
    ConstantsBase.Shooter constants;
    StatusSignal<Double> shooterPosition;


    public Shooter(RobotState robotState, double leftCurrentLimit, double rightCurrentLimit) {

        constants = robotState.getConstants().getShooterConstants();

        this.robotState = robotState;
        leftFlywheel = new TalonFX(constants.LEFT_FLYWHEEL_CAN, "drivetrain");
        rightFlywheel = new TalonFX(constants.RIGHT_FLYWHEEL_CAN, "drivetrain");

        leftVoltageVelocity = new VelocityVoltage(0, 0, true, 7.5, 0, false, false, false);
        rightVoltageVelocity = new VelocityVoltage(0, 0, true, 7.5, 0, false, false, false);
        leftTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0,0,0,0,false,false,false);
        rightTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0,0,0,0,false,false,false);



        TalonFXConfiguration leftConfigs = new TalonFXConfiguration();
        TalonFXConfiguration rightConfigs = new TalonFXConfiguration();

        leftConfigs.Slot0.kP = constants.FLYWHEEL_KP;
        leftConfigs.Slot0.kI = constants.FLYWHEEL_KI;
        leftConfigs.Slot0.kD = constants.FLYWHEEL_KD;
        leftConfigs.Slot0.kV = constants.LEFT_FLYWHEEL_KV;
        leftConfigs.Slot0.kS = constants.LEFT_FLYWHEEL_KS;
        
        rightConfigs.Slot0.kP = constants.RFLYWHEEL_KP;
        rightConfigs.Slot0.kI = constants.RFLYWHEEL_KI;
        rightConfigs.Slot0.kD = constants.RFLYWHEEL_KD;
        rightConfigs.Slot0.kV = constants.RIGHT_FLYWHEEL_KV;
        rightConfigs.Slot0.kS = constants.RIGHT_FLYWHEEL_KS;



        // leftConfigs.Voltage.PeakForwardVoltage = constants.FLYWHEEL_PEAK_VOLTAGE;
        // leftConfigs.Voltage.PeakReverseVoltage = -constants.FLYWHEEL_PEAK_VOLTAGE;
        // rightConfigs.Voltage.PeakForwardVoltage = constants.FLYWHEEL_PEAK_VOLTAGE;
        // rightConfigs.Voltage.PeakReverseVoltage = -constants.FLYWHEEL_PEAK_VOLTAGE;

        leftConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        leftConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftConfigs.TorqueCurrent.PeakForwardTorqueCurrent = leftCurrentLimit;
        leftConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -leftCurrentLimit;
        rightConfigs.TorqueCurrent.PeakForwardTorqueCurrent = rightCurrentLimit;
        rightConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -rightCurrentLimit;

        StatusCode rightStatus = StatusCode.StatusCodeNotInitialized;
        StatusCode leftStatus = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            leftStatus = leftFlywheel.getConfigurator().apply(leftConfigs);
            if (leftStatus.isOK()) break;
        }
        if(!leftStatus.isOK()) {
            System.out.println("Could not apply configs, error code: " + leftStatus.toString());
        }

        for (int i = 0; i < 5; ++i) {
            rightStatus = rightFlywheel.getConfigurator().apply(rightConfigs);
            if (rightStatus.isOK()) break;
        }
        if(!rightStatus.isOK()) {
            System.out.println("Could not apply configs, error code: " + rightStatus.toString());
        }

        shooterPosition = leftFlywheel.getPosition();

    }

    @Override
    public void updateState() {
        if (Math.abs(leftTorqueCurrentFOC.Velocity - leftFlywheel.getVelocity().getValue()) < 10 && 
                leftTorqueCurrentFOC.Velocity > (constants.LEFT_FLYWHEEL_SLOW_RPM / 60.0) && 
                Math.abs(rightTorqueCurrentFOC.Velocity - rightFlywheel.getVelocity().getValue()) < 10 && 
                rightTorqueCurrentFOC.Velocity > (constants.RIGHT_FLYWHEEL_SLOW_RPM / 60.0)) {

            robotState.setShooterOn(true);
            SmartDashboard.putBoolean("Shooter_on", true);
        } else {
            robotState.setShooterOn(false);
            SmartDashboard.putBoolean("Shooter_on", false);

        }

        shooterPosition.refresh();
        robotState.setShooterPos(leftFlywheel.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("Shooter_Left target speed", leftTorqueCurrentFOC.Velocity * 60);
        SmartDashboard.putNumber("Shooter_right target speed", rightTorqueCurrentFOC.Velocity * 60);
        SmartDashboard.putNumber("Shooter_Left speed RPM", leftFlywheel.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Shooter_Right speed RPM", rightFlywheel.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Shooter_ShootVolt", leftFlywheel.getMotorVoltage().getValue());
        SmartDashboard.putNumber("Shooter_commandedvolts", leftFlywheel.getSupplyVoltage().getValue());
        SmartDashboard.putBoolean("Shooter_isShooting", isShooting);
    }

    @Override
    public void teleop(RobotCommander commander) {

        shooterPosition.refresh();

        if (commander.armCommanded() == ArmCommanded.shotMap || commander.armCommanded() == ArmCommanded.close || commander.armCommanded() == ArmCommanded.protect || commander.armCommanded() == ArmCommanded.auton || commander.armCommanded() == ArmCommanded.sourceAuto || commander.armCommanded() == ArmCommanded.sourceAuto2 || commander.armCommanded() == ArmCommanded.sourceAutoRed) {
             leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity((constants.LEFT_FLYWHEEL_TARGET_RPM / 60.0)));
             rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity((constants.RIGHT_FLYWHEEL_TARGET_RPM / 60.0)).withFeedForward(20.0));
        }

        // } else if (commander.armCommanded() == ArmCommanded.trap && robotState.getShooterOnAmpTrap()){
        //     leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity(leftSlowSpeed));
        //     rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity(rightSlowSpeed));
            
        // }
        else if (commander.armCommanded() == ArmCommanded.zero){
            leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity((constants.LEFT_FLYWHEEL_IDLE_RPM / 60.0)));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity((constants.RIGHT_FLYWHEEL_IDLE_RPM / 60.0)));
        }
        else if (commander.armCommanded() == ArmCommanded.handoff && robotState.getShooterOnAmpTrap()){
            leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity((constants.LEFT_FLYWHEEL_SLOW_RPM / 60.0)));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity((constants.RIGHT_FLYWHEEL_SLOW_RPM / 60.0)));
        }
        else if (commander.armCommanded() == ArmCommanded.preload){
            leftFlywheel.setControl(leftVoltageVelocity.withVelocity((constants.RIGHT_FLYWHEEL_PRELOAD_RPM / 60.0)));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity((constants.RIGHT_FLYWHEEL_PRELOAD_RPM / 60.0)).withFeedForward(20.0));
        }
        else if (commander.armCommanded() == ArmCommanded.spitOut){
            leftFlywheel.setControl(leftVoltageVelocity.withVelocity((constants.LEFT_FLYWHEEL_SLOW_RPM / 60.0)));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity((constants.RIGHT_FLYWHEEL_SLOW_RPM/ 60.0)).withFeedForward(20.0));
        }
        else if (commander.armCommanded() == ArmCommanded.spitOut2){
            leftFlywheel.setControl(leftVoltageVelocity.withVelocity((2000.0 / 60.0)));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity((2000.0 / 60.0)).withFeedForward(20.0));
        }
        else if (commander.armCommanded() == ArmCommanded.mayaspit) {
            leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity((1800.0 / 60.0)));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity((1800.0 / 60.0)));
        } 
        else if (commander.armCommanded() == ArmCommanded.hailMary) {
            leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity((3700.0 / 60.0)));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity((2700.0 / 60.0)));
        } 
        else if (commander.armCommanded() == ArmCommanded.amp) {
            leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity((constants.LEFT_FLYWHEEL_SLOW_RPM / 60.0)));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity((constants.RIGHT_FLYWHEEL_SLOW_RPM / 60.0)));
        } 
        else {
            leftFlywheel.setVoltage(0);
            rightFlywheel.setVoltage(0);
        }

        if (commander.getRunFeeder() && !isShooting) {
            isShooting = true;
        }


        
    }

    @Override
    public void cameraLights() {
    }

    @Override
    public void reset() {
    }

    @Override
    public void init(RobotCommander commander) {
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    public TalonFX getLeftShooter(){
        return leftFlywheel;
    }

    public TalonFX getRightShooter(){
        return rightFlywheel;
    }
}