// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.StartStopTimer;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScriptedGalacticSearch extends SequentialCommandGroup {
  /** Creates a new ScriptedGalacticSearch. 
 * @param pathToRun*/
  public ScriptedGalacticSearch(IntakeSubsystem intakeSubsystem, Command pathToRun) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(
        pathToRun,
        new AutoIntakeAndSpin(intakeSubsystem),
        new StartStopTimer()
    ));

  }
}
