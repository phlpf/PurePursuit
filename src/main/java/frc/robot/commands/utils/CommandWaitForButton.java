// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandWaitForButton extends CommandBase {
  /** Creates a new CommandWaitForButton. */
  XboxController gamepad;
  int button;
  boolean wasReleased = false;
  public CommandWaitForButton(XboxController gamepad, int button) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gamepad = gamepad;
    this.button = button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gamepad.getRawButton(button) == false){
      wasReleased = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gamepad.getRawButton(button) && wasReleased;
  }
}
