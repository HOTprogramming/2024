package frc.robot.Subsystems;

import frc.robot.RobotCommander;
import frc.robot.RobotState;

import com.ctre.phoenix6.StatusCode;
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
    double leftHighSpeed = 69;
    double rightHighSpeed = 54;
    double leftSlowSpeed = 18;
    double rightSlowSpeed = 18;
    double leftIdleSpeed = 33;
    double rightIdleSpeed = 33;
    boolean isShooting = false;
    
    ConstantsBase.Shooter constants;


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


        leftConfigs.Voltage.PeakForwardVoltage = constants.FLYWHEEL_PEAK_VOLTAGE;
        leftConfigs.Voltage.PeakReverseVoltage = -constants.FLYWHEEL_PEAK_VOLTAGE;
        rightConfigs.Voltage.PeakForwardVoltage = constants.FLYWHEEL_PEAK_VOLTAGE;
        rightConfigs.Voltage.PeakReverseVoltage = -constants.FLYWHEEL_PEAK_VOLTAGE;

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

    }

    @Override
    public void updateState() {
        if (Math.abs(leftFlywheel.getVelocity().getValueAsDouble() - leftHighSpeed) < 4) {
            robotState.setShooterOn(true);
        } else {
            robotState.setShooterOn(false);

        }

        SmartDashboard.putNumber("Shooter_Left target speed", leftHighSpeed * 60);
        SmartDashboard.putNumber("Shooter_right target speed", rightHighSpeed * 60);
        SmartDashboard.putNumber("Shooter_Left speed RPM", leftFlywheel.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Shooter_Right speed RPM", rightFlywheel.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Shooter_ShootVolt", leftFlywheel.getMotorVoltage().getValue());
        SmartDashboard.putNumber("Shooter_commandedvolts", leftFlywheel.getSupplyVoltage().getValue());
        SmartDashboard.putBoolean("Shooter_isShooting", isShooting);
    }

    @Override
    public void enabled(RobotCommander commander) {
        if (commander.armCommanded() == ArmCommanded.shotMap || commander.armCommanded() == ArmCommanded.close || commander.armCommanded() == ArmCommanded.protect) {
             leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity(leftHighSpeed));
             rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity(rightHighSpeed));

        } else if (commander.armCommanded() == ArmCommanded.trap){
            leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity(leftSlowSpeed));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity(rightSlowSpeed));
            
        }
        else if (commander.armCommanded() == ArmCommanded.zero){
            leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity(leftIdleSpeed));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity(rightIdleSpeed));
        }
        else if (commander.armCommanded() == ArmCommanded.amp){
            leftFlywheel.setControl(leftTorqueCurrentFOC.withVelocity(leftSlowSpeed));
            rightFlywheel.setControl(rightTorqueCurrentFOC.withVelocity(rightSlowSpeed));
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
    public void disabled() {
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