package frc.robot.constants;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

public class kSwerve {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     *
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final GearRatio VEL_GEAR_RATIO = GearRatio.L3;
    public static final ModuleConfiguration VEL_GEAR_RATIO_CFG = VEL_GEAR_RATIO.getConfiguration();
    public static double MAX_VOLTAGE = 12.0;
    public static String CANIVORE_NAME = "rio";

    /**
     * The maximum velocity of the robot in meters per second.
     *
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * VEL_GEAR_RATIO_CFG.getDriveReduction()
            * VEL_GEAR_RATIO_CFG.getWheelDiameter() * Math.PI;

    public static final double MAX_ACCELERATION = 4; //1.7;
    

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.445; 
    
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.445;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 13.96;

    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(118.37);
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(102.99);
    public static final double REAR_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(74.26);
    public static final double REAR_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(339.77);

    public static final double SWERVE_ALLOWED_OFFSET = 1.0;

    public static double SWERVE_CORRECTION_SPEED = 0.05 * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
}
