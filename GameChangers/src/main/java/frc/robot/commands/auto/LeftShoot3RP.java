// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.RamseteFollower;
import frc.robot.commands.StopFlyWheel;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HopperOmniSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftShoot3RP extends SequentialCommandGroup {
  /** Creates a new LeftShoot3RP. */
  public LeftShoot3RP(DriveTrainMain driveTrain, VisionSystem visionSubsystem, FlyWheelSubsystem flyWheel, 
  HopperOmniSubsystem hopperOmni, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    RamseteFollower initialPath = (RamseteFollower)Drive.getAutonomousCommand("paths/LeftRP1.wpilib.json",true);
    addCommands(
      new BeginningShoot3(driveTrain, visionSubsystem, flyWheel, hopperOmni, intake, initialPath),
      initialPath,
      new StopFlyWheel(flyWheel)
    );
  }
}
