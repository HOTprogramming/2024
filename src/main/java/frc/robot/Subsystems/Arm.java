package frc.robot.Subsystems;

import static frc.robot.Constants.ArmConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.RobotCommander;

public class Arm implements SubsystemBase{
    RobotState robotState;

    TalonFX armMotor;
    MotionMagicVoltage motionMagic;

    public Arm(RobotState robotState) {
        this.robotState = robotState;
        armMotor = new TalonFX(ARM_CAN);
        motionMagic = new MotionMagicVoltage(0);

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        TalonFXConfiguration configs = new TalonFXConfiguration();

        MotionMagicConfigs mm = configs.MotionMagic;
        mm.MotionMagicCruiseVelocity = ARM_CRUISE_VELOCITY; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = ARM_ACCELERATION; // Take approximately 0.5 seconds to reach max vel
        mm.MotionMagicJerk = ARM_JERK;

        Slot0Configs slot0 = configs.Slot0;
        slot0.kP = ARM_KP;
        slot0.kI = ARM_KI;
        slot0.kD = ARM_KD;
        slot0.kV = ARM_KV;
        slot0.kS = ARM_KS;

        for(int i = 0; i < 5; ++i) {
            status = armMotor.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    public void updateState() {

    }

    public void enabled(RobotCommander commander) {
    
        SmartDashboard.putNumber("position", armMotor.getPosition().getValueAsDouble());
        armMotor.setControl(motionMagic.withPosition(commander.getRunArm()).withSlot(0));
    }

    public void disabled() {

    }

    public void reset() {

    }

    @Override
    public void init(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }
}
