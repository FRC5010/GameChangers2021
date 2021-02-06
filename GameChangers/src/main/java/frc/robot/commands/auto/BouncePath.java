// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.StartStopTimer;
import frc.robot.mechanisms.Drive;


public class BouncePath extends SequentialCommandGroup {
  /** Creates a new BouncePath. */
  public BouncePath() {
    String path1 = "paths/Bounce1.wpilib.json";
    String path2 = "paths/Bounce2NR.wpilib.json";
    String path3 = "paths/Bounce3NR.wpilib.json";
    String path4 = "paths/Bounce4NR.wpilib.json";
    addCommands(new ParallelDeadlineGroup(
        new SequentialCommandGroup(
            Drive.getAutonomousCommand(path1, true),
            Drive.getAutonomousCommand(path2),
            Drive.getAutonomousCommand(path3),
            Drive.getAutonomousCommand(path4)
            ), 
            new StartStopTimer()));
  }
}
