/*package frc.robot.Subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import frc.robot.RobotCommander;
import frc.robot.RobotState;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter implements SubsystemBase {
    RobotState robotState;
    TalonFX leftFlywheel;
    TalonFX rightFlywheel;
    TalonFX feeder;
    VelocityVoltage leftVoltageVelocity;
    VelocityVoltage rightVoltageVelocity;
    double leftTargetSpeed = 10;
    double rightTargetSpeed = 10;

    public Shooter(RobotState robotState) {
        this.robotState = robotState;
        leftFlywheel = new TalonFX(LEFT_FLYWHEEL_CAN, "drivetrain");
        rightFlywheel = new TalonFX(RIGHT_FLYWHEEL_CAN, "drivetrain");
        feeder = new TalonFX(FEEDER_CAN, "drivetrain");

        leftVoltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
        rightVoltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

        StatusCode rightStatus = StatusCode.StatusCodeNotInitialized;
        StatusCode leftStatus = StatusCode.StatusCodeNotInitialized;

        TalonFXConfiguration leftConfigs = new TalonFXConfiguration();
        TalonFXConfiguration rightConfigs = new TalonFXConfiguration();

        leftConfigs.Slot0.kP = SHOOTER_KP;
        leftConfigs.Slot0.kI = SHOOTER_KI;
        leftConfigs.Slot0.kD = SHOOTER_KD;
        leftConfigs.Slot0.kV = LEFT_SHOOTER_KV;
        leftConfigs.Slot0.kS = LEFT_SHOOTER_KS;
        
        rightConfigs.Slot0.kP = SHOOTER_KP;
        rightConfigs.Slot0.kI = SHOOTER_KI;
        rightConfigs.Slot0.kD = SHOOTER_KD;
        rightConfigs.Slot0.kV = RIGHT_SHOOTER_KV;
        rightConfigs.Slot0.kS = RIGHT_SHOOTER_KS;

        leftConfigs.Voltage.PeakForwardVoltage = SHOOTER_PEAK_VOLTAGE;
        leftConfigs.Voltage.PeakReverseVoltage = -SHOOTER_PEAK_VOLTAGE;
        rightConfigs.Voltage.PeakForwardVoltage = SHOOTER_PEAK_VOLTAGE;
        rightConfigs.Voltage.PeakReverseVoltage = -SHOOTER_PEAK_VOLTAGE;

        leftConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        leftConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;


    for (int i = 0; i < 5; ++i) {
      leftStatus = leftFlywheel.getConfigurator().apply(leftConfigs);
      rightStatus = rightFlywheel.getConfigurator().apply(rightConfigs);
      if (leftStatus.isOK() && rightStatus.isOK()) break;
    }
    if(!leftStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + leftStatus.toString());
    }
    if(!rightStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + rightStatus.toString());
    }
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
        leftFlywheel.setControl(leftVoltageVelocity.withVelocity(leftTargetSpeed));
        rightFlywheel.setControl(rightVoltageVelocity.withVelocity(rightTargetSpeed));

        if (commander.increaseLeftTargetSpeed()) {
            leftTargetSpeed += TARGET_SPEED_INCREMENT;
        }
        if (commander.decreaseLeftTargetSpeed()) {
            leftTargetSpeed -= TARGET_SPEED_INCREMENT;
        }
        if (commander.increaseRightTargetSpeed()) {
            rightTargetSpeed += TARGET_SPEED_INCREMENT;
        }
        if (commander.decreaseRightTargetSpeed()) {
            rightTargetSpeed -= TARGET_SPEED_INCREMENT;
        }
            SmartDashboard.putNumber("Left target speed", leftTargetSpeed * 60);
            SmartDashboard.putNumber("right target speed", rightTargetSpeed * 60);
            SmartDashboard.putNumber("Left speed RPM", leftFlywheel.getVelocity().getValueAsDouble() * 60);
            SmartDashboard.putNumber("Right speed RPM", rightFlywheel.getVelocity().getValueAsDouble() * 60);
            SmartDashboard.putNumber("left DC", leftFlywheel.getDutyCycle().getValueAsDouble());
            SmartDashboard.putNumber("Left Voltage", leftFlywheel.getMotorVoltage().getValueAsDouble());

        if (commander.getRunShooter()) {
            feeder.set(FEEDER_SPEED);
        } else {
            feeder.set(0);
        }
    }

    @Override
    public void disabled() {
    }

    @Override
    public void reset() {
        rightTargetSpeed = 10;
        leftTargetSpeed = 10;
    }
<<<<<<< Updated upstream

    @Override
    public void init(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    public TalonFX getLeftShooter(){
        return leftFlywheel;
    }

    public TalonFX getRightShooter(){
        return rightFlywheel;
    }

    public TalonFX getFeederMotor(){
        return feeder;
    }
}
=======
}
*/
>>>>>>> Stashed changes
