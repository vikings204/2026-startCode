package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot.ControlMode;

import static edu.wpi.first.math.util.Units.inchesToMeters;

@SuppressWarnings("unused")
public final class Constants {
    public static final class Controller {
        public static final int DRIVER_PORT = 1;
        public static final int OPERATOR_PORT = 2;
        public static final double DEADBAND = 0.1;
        public static final ControlMode DEFAULT_CONTROL_MODE = ControlMode.SINGLE;

        // will eventually have keybinds and stuff
    }

    public static final class Swerve {
        public static final double SPEED_MULTIPLIER = 1.0;
        public static final double CONTROLLER_RAMP_DEGREE = 1;

        public static final double ANGLE_PID_FF = 0.0;
        public static final double DRIVE_PID_P = 1.0;
        public static final double DRIVE_PID_I = 0.0;
        public static final double DRIVE_PID_D = 0.01;
        public static final double DRIVE_PID_FF = 0.0;
        public static final double DRIVE_FF_S = 0.667;
        public static final double DRIVE_FF_V = 2.44;
        public static final double DRIVE_FF_A = 0.27;
        public static final double ANGLE_PID_P = 0.01;
        public static final double ANGLE_PID_I = 0.0;
        public static final double ANGLE_PID_D = 0.0;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = inchesToMeters(23); // same as wheelbase because it is a square
        public static final double WHEEL_BASE = inchesToMeters(23);
        public static final double WHEEL_DIAMETER_REAL = inchesToMeters(4);
        public static final double WHEEL_DIAMETER = inchesToMeters(3.81);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double DRIVE_GEAR_RATIO = 8.14;
        public static final double ANGLE_GEAR_RATIO = (150.0 / 7.0);
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        /* Drive Motor Conversion Factors */
        public static final double DRIVE_POSITION_CONVERSION_FACTOR = (WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_RATIO;
        public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR / 60.0;
        public static final double ANGLE_POSITION_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

        /* Swerve Voltage Compensation */
        public static final double VOLTAGE_COMPENSATION = 12.0;

        /* Swerve Current Limiting */
        public static final int DRIVE_CURRENT_LIMIT = 40;//30;
        public static final int ANGLE_CURRENT_LIMIT = 10;//5;

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 4.5; // meters per second
        public static final double MAX_ANGULAR_VELOCITY = 8; // radians per second

        /* Neutral Modes */
        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean DRIVE_INVERT = true;
        public static final boolean ANGLE_INVERT = true;
        public static final boolean canCoderInvert = false;


        public static final boolean GYRO_INVERT = true; // Always ensure Gyro is CCW+ CW-
        public static final int PIGEON2_ID = 9;

        /* Module Specific Constants */

        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 21;
            public static final int CAN_CODER_ID = 31;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(.919434);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 12;
            public static final int ANGLE_MOTOR_ID = 22;
            public static final int CAN_CODER_ID = 32;

            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(.12793);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 20;
            public static final int CAN_CODER_ID = 30;

            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(.077881);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 13;
            public static final int ANGLE_MOTOR_ID = 23;
            public static final int CAN_CODER_ID = 33;

            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(.33374);
        }
    }

    public static final class Auto {
        public static final RobotConfig config;
        static {
            RobotConfig tempConfig = null;
            try {
                tempConfig = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                e.printStackTrace();
            }
            config = tempConfig;
        }

        public static final PPHolonomicDriveController PATH_FOLLOWER_CONFIG =  new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        );
    }

    public static final class Vision {
        public static final boolean VISION_ENABLED = true;
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
        public static final double TRUST_VISION_STANDARD_DEVIATION = 0.3;
        public static final double IGNORE_VISION_STANDARD_DEVIATION = 1.5;

        /**
         * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
         */
        public static final Vector<N3> STATE_STANDARD_DEVIATIONS = VecBuilder.fill(TRUST_VISION_STANDARD_DEVIATION, TRUST_VISION_STANDARD_DEVIATION, TRUST_VISION_STANDARD_DEVIATION);

        public static final double MAX_SPEED_FOR_STD_DEV = 1.0;
        public static final double MIN_SPEED_FOR_STD_DEV = 0.1;
    }

    public static final class Intake {
        public static final double PID_P = .25;
        public static final int LEFT_MOTOR_ID = 7;
        public static final int RIGHT_MOTOR_ID = 8;
        public static final int IntakeSpinMotor_ID = 9;
        public static final double VOLTAGE_COMPENSATION = 12.0;
        public static final int CURRENT_LIMIT = 40;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
        public static final boolean LEFT_INVERT = false;
        public static final boolean RIGHT_INVERT = !LEFT_INVERT;
        public static final double POSITION_CONVERSION_FACTOR = 12;
        public static final int AUTOMATIC_ZERO_CURRENT = 15;
        
        public enum Positions {
            INTAKE(0.96),
            Auto(0.96),
            ZERO(0);

            public final double position;
            Positions(double p) {
                this.position = p;
            }
        }
    }

    public static final class Shooter {
        public static final int MAIN_MOTOR_ID = 51;
        public static final int KICK_MOTOR_ID = 52;
        public static final int VECTOR_MOTOR_ID = 53;
    }

    public static final class Climber {
        public static final double VOLTAGE_COMPENSATION = 12.0;
        public static final int CURRENT_LIMIT = 40;
        public static final boolean INVERT = false;
        public static final double PID_P = 10.0;
        public static final int MOTOR_ID = 143;

        /* Neutral Modes */
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    }
    
}