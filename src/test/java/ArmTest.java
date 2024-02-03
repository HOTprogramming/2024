// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.sim.TalonFXSimState;

// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.simulation.DriverStationSim;
// import frc.robot.RobotState;
// import frc.robot.Subsystems.Arm;

// import static org.junit.jupiter.api.Assertions.*;

// import java.beans.Transient;

// import org.junit.jupiter.api.*;

// public class ArmTest implements AutoCloseable {
//    static final double DELTA = 1e-3; // acceptable deviation range

//    TalonFXSimState armSim;

//    Arm arm;

//    @Override
//    public void close() {
//    }

//    @BeforeEach
//    public void constructDevices() {
//       assert HAL.initialize(500, 0);

//       /* create the TalonFX */
//       arm = new Arm(new RobotState());
//       armSim = arm.getArmMotor().getSimState();

//       /* enable the robot */
//       DriverStationSim.setEnabled(true);
//       DriverStationSim.notifyNewData();

//       /* delay ~100ms so the devices can start up and enable */
//       Timer.delay(0.100);
//    }

//    @AfterEach
//    void shutdown() {
        
//    }

//    @Test
//    public void robotIsEnabled() {
//       /* verify that the robot is enabled */
//       assertTrue(DriverStation.isEnabled());
//    }

//    @Test
//    public void checkConstructor(){
//     assertNotEquals(arm, null);
//    }

//    @Test
//    public void armMotorTurnsOn() {
//       /* set the voltage supplied by the battery */
//       armSim.setSupplyVoltage(RobotController.getBatteryVoltage());

//       var dutyCycle = arm.getArmMotor().getDutyCycle();

//       /* wait for a fresh duty cycle signal */
//       dutyCycle.waitForUpdate(0.100);
//       /* verify that the motor output is zero */
//       assertEquals(dutyCycle.getValue(), 0.0, DELTA);

//       /* request 100% output */
//       arm.getArmMotor().set(1);

//       /* wait for the control to apply */
//       Timer.delay(0.020);

//       /* wait for a new duty cycle signal */
//       dutyCycle.waitForUpdate(0.100);

//       /* verify that the motor output is 1.0 */
//       assertEquals(dutyCycle.getValue(), 1.0, DELTA);
//    }

// //    @Test
// //    public void confirmArmEdges(){

// //    }
// }