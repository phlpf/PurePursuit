package frc.robot.constants;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

public class kSwerve {
    public static final String CANIVORE_NAME = "rio";

    public static final double WHEEL_DIAMETER_METERS = 0.10033;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0) // mk4 drive reduction
            * WHEEL_DIAMETER_METERS * Math.PI;
    public static final double MAX_ACCELERATION = 4; //1.7;
    public static double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 13.96;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.445;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.445;

    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 13.2;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 251;
    public static final double REAR_RIGHT_MODULE_STEER_OFFSET = 28.7;
    public static final double REAR_LEFT_MODULE_STEER_OFFSET = 344.3;

    public static final double SWERVE_ALLOWED_OFFSET = 1.0;
    public static double SWERVE_CORRECTION_SPEED = 0.05 * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    public static final double DRIVE_GEAR_RATIO = 6.75; // 6.75:1
    public static final double ANGLE_GEAR_RATIO = 12.8; // 12.8:1

    // TODO: Copied values directly from 364 code, for MK3 modules
    public static final double DRIVE_MOTOR_KS = (0.667 / 12);
    public static final double DRIVE_MOTOR_KV = (2.44 / 12);
    public static final double DRIVE_MOTOR_KA = (0.27 / 12);

    public static final double DRIVE_MOTOR_KP = 0.1;
    public static final double DRIVE_MOTOR_KI = 0.0;
    public static final double DRIVE_MOTOR_KD = 0.0;
    public static final double DRIVE_MOTOR_KF = 0.0;

    public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
    public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0;
    public static final double DRIVE_CONTINUOUS_CURRENT_LIMIT = 40;
    public static final double DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;

    public static final double ANGLE_MOTOR_KP = 0.3;
    public static final double ANGLE_MOTOR_KI = 0.0;
    public static final double ANGLE_MOTOR_KD = 1.0;
    public static final double ANGLE_MOTOR_KF = 0.0;

    public static final double ANGLE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final double ANGLE_PEAK_CURRENT_LIMIT = 60;
    public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
}
