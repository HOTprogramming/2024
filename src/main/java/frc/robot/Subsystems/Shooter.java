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


public class Shooter implements SubsystemBase {
    RobotState robotState;
    TalonFX flywheel;
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    

    public Shooter(RobotState robotState) {
        this.robotState = robotState;
        flywheel = new TalonFX(FLYWHEEL_CAN);
    }

    public void init(){
        TalonFXConfiguration configs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        configs.Slot0.kP = SHOOTERKP;//0.11; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = SHOOTERKI; //0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = SHOOTERKD; //0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = SHOOTERKV; //0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;
        
        
    
     
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = flywheel.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    public void shootAtSpeed(int speed) {
        
    }

    @Override
    public void updateState() {
        if (flywheel.getVelocity().getValueAsDouble() >= (FLYWHEEL_MAX_SPEED - FLYWHEEL_MAX_VELOCITY_ERROR)) {
            robotState.setShooterOn(true);
        } else {
            robotState.setShooterOn(false);
        }
    }
    
    @Override
    public void enabled(RobotCommander commander) {
        if (commander.getRunShooter()) {
            flywheel.setControl(m_voltageVelocity.withVelocity(FLYWHEEL_MAX_SPEED));;
        } else {
            flywheel.set(0);
        }
    }

    @Override
    public void disabled() {
        flywheel.stopMotor();
    }

    @Override
    public void reset() {
        flywheel.stopMotor();
    }
}
