// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drives.DriveCommand;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kControl;
import frc.robot.constants.kSwerve;
import frc.robot.subsystems.*;
import frc.robot.utils.AutoUtil;
import frc.robot.utils.PurePursuitHelper;

public class RobotContainer {

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    private final Drives drives = new Drives();
    private final PurePursuitHelper purePursuit = new PurePursuitHelper(drives);
    private final DriveCommand pursue = new DriveCommand(drives, 
                                                () -> purePursuit.getXSpeed() * kSwerve.MAX_VELOCITY_METERS_PER_SECOND,
                                                () ->  purePursuit.getYSpeed() * kSwerve.MAX_VELOCITY_METERS_PER_SECOND, 
                                                () -> purePursuit.getRotSpeed() * kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                                                () -> purePursuit.isFinished(),
                                                () -> purePursuit.update()
                                        );

    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drives.setDefaultCommand(new DriveCommand(
                drives,
                        () -> -modifyAxis(driverController.getLeftY()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * (1 - (modifyAxis(driverController.getLeftTriggerAxis()) * 0.9)),
                        () -> -modifyAxis(driverController.getLeftX()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * (1 - (modifyAxis(driverController.getLeftTriggerAxis()) * 0.9)),
                        () -> -modifyAxis(driverController.getRightX()) * kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * (1 - (modifyAxis(driverController.getLeftTriggerAxis()) * 0.9))
        ));

        // Configure the button bindings
        configureDriverControllerBindings();
        configureOperatorControllerBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureDriverControllerBindings() {
        // Back button zeros the gyroscope
        new Button(driverController::getBackButton)
                        .whenPressed(drives::zeroGyroscope);
        // new Button(driverController::getStartButton);
        new Button(driverController::getAButton)
                        .whenPressed(pursue);
        new Button(driverController::getBButton)
                        .whenPressed(() -> pursue.cancel());


        // POV
        // new POVButton(driverController, 0);
        // new POVButton(driverController, 90);
        // new POVButton(driverController, 180);
        // new POVButton(driverController, 270);

        // Bumpers
        // Joystick Buttons
        // new Button(driverController::getRightStickButton);
        // new Button(driverController::getLeftStickButton);

        // Triggers
        // new Trigger(() -> driverController.getRightTriggerAxis() > 0.5);
        // new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5);
    }

    private void configureOperatorControllerBindings() {
        // Start/Back
    }

    public void resetSubsystems() {
        drives.zeroGyroscope();
    }

    public static double modifyAxis(double rawValue) {
        // Deadband
        double deadband = 0.05;
        double computedValue = rawValue;
        if (Math.abs(computedValue) > deadband) {
            if (computedValue > 0.0) {
                computedValue = (computedValue - deadband) / (1.0 - deadband);
            } else {
                computedValue = (computedValue + deadband) / (1.0 - deadband);
            }
        } else {
            computedValue = 0.0;
        }

        // Square the axis
        computedValue = Math.copySign(computedValue * computedValue, computedValue);

        return computedValue;
    }

    public void runAutonomousRoutine(AutoUtil.Routine routine) {
      
    }
    public void setLEDs(int pattern){
    }
}
