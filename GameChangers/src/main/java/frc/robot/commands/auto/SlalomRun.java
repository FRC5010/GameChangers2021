// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.StartStopTimer;
import frc.robot.mechanisms.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlalomRun extends SequentialCommandGroup {
  /** Creates a new SlalomRun. */
  public SlalomRun() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    String path = "paths/SlalomFewerPoints.wpilib.json";
    addCommands(new ParallelDeadlineGroup(Drive.getAutonomousCommand(path, true), new StartStopTimer()));
    //addCommands(new ParallelDeadlineGroup(Drive.getAutonomousCommand(path), new StartStopTimer()));
  }
}
