// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.index.CommandMoveIndex;
import frc.robot.constants.kControl;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterRPMS;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ComplexShootBalls extends SequentialCommandGroup {
  /** Creates a new ComplexShootBalls. */
  public ComplexShootBalls(Shooter shooter, Index index, Acquisition acquisition, int balls, ShooterRPMS rpms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> acquisition.extendArms()),
      new InstantCommand(() -> acquisition.setRollerRPM(0)),
      new CommandRunShooter(shooter, rpms),
      new CommandMoveIndex(index, balls * kControl.INDEX_ONE_BALL_ROTATIONS),
      new InstantCommand(() -> index.setBallsIndexed(Math.max(0, index.getBallsIndexed() - balls))),
      new InstantCommand(() -> shooter.setPercentOut(0)),
      new InstantCommand(() -> index.runPercentOut(0)),
      new InstantCommand(() -> acquisition.setRollerRPM(kControl.ACQUISITION_RPMS))
    );
  }
}
