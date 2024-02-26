package frc.robot.Subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

public class CameraMeasurment {
    private String name;
    public CameraMeasurment(String string) {
        name = string;
    }
        Nat<N3> rows = new Nat<N3>() {

        @Override
        public int getNum() {
            return 3;
        }
        
    };

    private Matrix<N3, N1> sdtDeviation = VecBuilder.fill(1, 1, 1);
    private double ambiguity = 0.0;
    private Pose2d pose = new Pose2d();
    private double timestamp = -1.0;

    public Matrix<N3, N1> getSdtDeviation() {
        return sdtDeviation;
    }
    public void setSdtDeviation(int row, int col, double value) {
        this.sdtDeviation.set(row, col, value);
    }
    public double getTimestamp() {
        return timestamp;
    }
    public void setTimestamp(double timestamp) {
        this.timestamp = timestamp;
    }
    public Pose2d getPose() {
        return pose;
    }
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }
    public double getAmbiguity() {
        return ambiguity;
    }
    public void setAmbiguity(double ambiguity) {
        this.ambiguity = ambiguity;
    }
}
