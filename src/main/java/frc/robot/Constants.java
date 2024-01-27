package frc.robot;


public final class Constants {

    public final class ShooterConstants {
        public static final int RIGHT_FLYWHEEL_CAN = 1;
        public static final int LEFT_FLYWHEEL_CAN = 2;
        public static final int FEEDER_CAN = 3;

        public static final double TARGET_SPEED_INCREMENT = 3;
        public static final double FEEDER_SPEED = 0.05;

        public static final double FLYWHEEL_MAX_SPEED = 0.05; // percent of full speed
        public static final double FLYWHEEL_MAX_VELOCITY_ERROR = .0005; // percent of full speed

        public static final double SHOOTER_KP = 0.11;
        public static final double SHOOTER_KI = 0.5;
        public static final double SHOOTER_KD = 0.0001;
        public static final double SHOOTER_KV = 0.12;
    }

    public final class DrivetrainConstants {
        public static final int DRIVE_CAN = 10;
        
    }

    public final class ArmConstants {
        public static final int ARM_CAN = 22;

        public static final double ARM_KP = 5;
        public static final double ARM_KI = 0;
        public static final double ARM_KD = 0.1;
        public static final double ARM_KV = 0.12;
        public static final double ARM_KS = 0.25;

        public static final double ARM_CRUISE_VELOCITY = 5;
        public static final double ARM_ACCELERATION = 10;
        public static final double ARM_JERK = 50;
    }
}
