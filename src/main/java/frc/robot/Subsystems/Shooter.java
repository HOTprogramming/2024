package frc.robot.Subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import frc.robot.RobotCommander;
import frc.robot.RobotState;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter implements SubsystemBase {
    RobotState robotState;
    TalonFX leftFlywheel;
    TalonFX rightFlywheel;
    TalonFX feeder;
    VelocityTorqueCurrentFOC leftVoltageVelocity;
    VelocityTorqueCurrentFOC rightVoltageVelocity;
    double leftTargetSpeed = 0;
    double rightTargetSpeed = 0;
    double tempSpeed = 0;


    public Shooter(RobotState robotState) {
        this.robotState = robotState;
        leftFlywheel = new TalonFX(LEFT_FLYWHEEL_CAN, "drivetrain");
        rightFlywheel = new TalonFX(RIGHT_FLYWHEEL_CAN, "drivetrain");
        feeder = new TalonFX(FEEDER_CAN, "drivetrain");

        // velocity = new VelocityVoltage(FLYWHEEL_MAX_VELOCITY_ERROR, FLYWHEEL_MAX_SPEED, false, FEEDER_SPEED, FEEDER_CAN, false, false, false)

        // leftVoltageVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, FEEDER_CAN, true, false, false);
        // rightVoltageVelocity = new VelocityTorqueCurrentFOC(0, 0, true, 0, 0, false, false, false);

        // StatusCode rightStatus = StatusCode.StatusCodeNotInitialized;
        // StatusCode leftStatus = StatusCode.StatusCodeNotInitialized;

        // TalonFXConfiguration configs = new TalonFXConfiguration();

        // configs.Slot0.kP = SHOOTER_KP; //0.11
        // configs.Slot0.kI = SHOOTER_KI; //0.5
        // configs.Slot0.kD = SHOOTER_KD; //0.0001
        // configs.Slot0.kV = SHOOTER_KV; //0.12

        // configs.Voltage.PeakForwardVoltage = 8;
        // configs.Voltage.PeakReverseVoltage = -8;

    // for (int i = 0; i < 5; ++i) {
    //   leftStatus = leftFlywheel.getConfigurator().apply(configs);
    //   rightStatus = rightFlywheel.getConfigurator().apply(configs);
    //   if (leftStatus.isOK() && rightStatus.isOK()) break;
    // }
    // if(!leftStatus.isOK()) {
    //   System.out.println("Could not apply configs, error code: " + leftStatus.toString());
    // }

    // for (int i  = 0;i < 5; ++i) {
    //   rightStatus = rightFlywheel.getConfigurator().apply(configs);
    //   if (rightStatus.isOK()) break;
    // }
    // if(!rightStatus.isOK()) {
    //   System.out.println("Could not apply configs, error code: " + rightStatus.toString());
    // }
    }

    @Override
    public void updateState() {
        if (rightFlywheel.getVelocity().getValueAsDouble() >= (FLYWHEEL_MAX_SPEED - FLYWHEEL_MAX_VELOCITY_ERROR)) {
            robotState.setShooterOn(true);
        } else {
            robotState.setShooterOn(false);
        }
    }

    @Override
    public void enabled(RobotCommander commander) {

        // leftFlywheel.setControl(leftVoltageVelocity.withVelocity(leftTargetSpeed));
        // rightFlywheel.setControl(rightVoltageVelocity.withVelocity(rightTargetSpeed));

        // if (commander.increaseLeftTargetSpeed()) {
        //     leftTargetSpeed += TARGET_SPEED_INCREMENT;
        // }
        // if (commander.decreaseLeftTargetSpeed()) {
        //     leftTargetSpeed -= TARGET_SPEED_INCREMENT;
        // }
        // if (commander.increaseRightTargetSpeed()) {
        //     rightTargetSpeed += TARGET_SPEED_INCREMENT;
        // }
        // if (commander.decreaseRightTargetSpeed()) {
        //     rightTargetSpeed -= TARGET_SPEED_INCREMENT;
        // }
        //     SmartDashboard.putNumber("Left target speed", leftTargetSpeed);
        //     SmartDashboard.putNumber("right target speed", rightTargetSpeed);
        //     SmartDashboard.putNumber("Left speed", leftFlywheel.getVelocity().getValueAsDouble());
        //     SmartDashboard.putNumber("Right speed", rightFlywheel.getVelocity().getValueAsDouble());
        // leftFlywheel.set(.5);
        // rightFlywheel.set(.5);
        if (commander.increaseLeftTargetSpeed()) {
            leftFlywheel.setVoltage(-8);
            rightFlywheel.setVoltage(14);
        } else {
            leftFlywheel.setVoltage(0);
            rightFlywheel.setVoltage(0);
        }

        if (commander.getRunShooter()) {
            feeder.set(FEEDER_SPEED);
        } else {
            feeder.set(0);
        }

        // leftFlywheel.set(1);
        // rightFlywheel.set(1);
    }

    @Override
    public void disabled() {
        
    }

    @Override
    public void reset() {
        rightFlywheel.stopMotor();
        leftFlywheel.stopMotor();
        rightTargetSpeed = 0;
        leftTargetSpeed = 0;
    }

    @Override
    public void init(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }
}
