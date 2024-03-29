// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.RamseteFollower;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HopperOmniSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot3Trench3 extends SequentialCommandGroup {
  /** Creates a new Shoot3Trench3. */
  public Shoot3Trench3(DriveTrainMain driveTrain, VisionSystem visionSubsystem, FlyWheelSubsystem flyWheel, 
  HopperOmniSubsystem hopperOmni, IntakeSubsystem intake) {
    RamseteFollower trench1 = (RamseteFollower)Drive.getAutonomousCommand("paths/Trench1.wpilib.json", false);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BeginningShoot3(driveTrain, visionSubsystem, flyWheel, hopperOmni, intake, trench1),
      new ParallelDeadlineGroup(
        trench1,
        new IntakeBalls(intake, 0.8)
      ),
      

      new ParallelDeadlineGroup(
         Drive.getAutonomousCommand("paths/Trench2.wpilib.json", false),
         new IntakeBalls(intake, 0.8)
       ),
      
       new EndShootAuto(driveTrain, visionSubsystem, flyWheel, hopperOmni, intake)
    );
  }
}
