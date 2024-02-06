package frc.robot;

public final class Constants {

    public final class ShooterConstants {
        public static final int SHOOTER1_CAN = 11;
        public static final double SHOOTER1_MAX_SPEED = 5; // percent of full speed
        public static final double SHOOTER1_MAX_VELOCITY_ERROR = .005; // percent of full speed
        public static final double SHOOTER1KP = 0.11;
        public static final double SHOOTER1KI = 0.5;
        public static final double SHOOTER1KD = 0.0001;
        public static final double SHOOTER1KV = 0.12;

        public static final int SHOOTER2_CAN = 12;
        public static final double SHOOTER2_MAX_SPEED = 5; // percent of full speed
        public static final double SHOOTER2_MAX_VELOCITY_ERROR = .005; // percent of full speed
        public static final double SHOOTER2KP = 0.11;
        public static final double SHOOTER2KI = 0.5;
        public static final double SHOOTER2KD = 0.0001;
        public static final double SHOOTER2KV = 0.12;

        public static final int FLOORINTAKE_CAN = 14;
        public static final double FLOORINTAKE_MAX_SPEED = 5; // percent of full speed
        public static final double FLOORINTAKE_MAX_VELOCITY_ERROR = .005; // percent of full speed
        public static final double FLOORINTAKEKP = 0.11;
        public static final double FLOORINTAKEKI = 0.5;
        public static final double FLOORINTAKEKD = 0.0001;
        public static final double FLOORINTAKEKV = 0.12;

        public static final int SHOOTERINTAKE_CAN = 13;
        public static final double SHOOTERINTAKE_MAX_SPEED = 5; // percent of full speed
        public static final double SHOOTERINTAKE_MAX_VELOCITY_ERROR = .005; // percent of full speed
        public static final double SHOOTERINTAKEKP = 0.11;
        public static final double SHOOTERINTAKEKI = 0.5;
        public static final double SHOOTERINTAKEKD = 0.0001;
        public static final double SHOOTERINTAKEKV = 0.12;
    }

    public final class DrivetrainConstants {
        public static final int DRIVE_CAN = 1;
        
    }

    public final class ArmConstants {
        public static final int CANCODER_CAN = 44;
        public static final int ARM_CAN = 9;
        public static final double CRUISEVELOCITY = 400;
        public static final double ACCELERATION = 400;
        public static final double JERK = 2000;
        public static final double ARMKP = 380;
        public static final double ARMKI = 0;
        public static final double ARMKD = 0;
        public static final double ARMKV = 0.8;
        public static final double ARMKS = 0.1;
        
    }
}
