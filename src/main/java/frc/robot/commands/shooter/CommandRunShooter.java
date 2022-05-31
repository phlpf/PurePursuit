// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterRPMS;

public class CommandRunShooter extends CommandBase {
  /** Creates a new CommandRunShooter. */
  Shooter shooter;
  double RPMFront;
  double RPMBack;
  boolean isInstant;
  public CommandRunShooter(Shooter shooter, ShooterRPMS rpms) {
    this.shooter = shooter;
    this.RPMFront = rpms.RPMFront;
    this.RPMBack = rpms.RPMBack;
    this.isInstant = false;
  }
  public CommandRunShooter(Shooter shooter, ShooterRPMS rpms, boolean isInstant) {
    this.shooter = shooter;
    this.RPMFront = rpms.RPMFront;
    this.RPMBack = rpms.RPMBack;
    this.isInstant = isInstant;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("shooter/isRPM", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocityFront(RPMFront);
    shooter.setVelocityBack(RPMBack);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("shooter/isRPM", true);
    System.out.println(shooter.getVelocityBack() + " + " + shooter.getVelocityFront());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(shooter.getVelocityFront() - RPMFront) < 50 && Math.abs(shooter.getVelocityBack() - RPMBack) < 50) || isInstant;
  }
}
