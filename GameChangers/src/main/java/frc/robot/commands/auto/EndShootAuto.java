// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimWithVision;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.ShootBall;
import frc.robot.commands.StopFlyWheel;
import frc.robot.commands.Timer;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HopperOmniSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EndShootAuto extends SequentialCommandGroup {
  /** Creates a new EndShootAuto. */
  public EndShootAuto(DriveTrainMain driveTrain, VisionSystem visionSubsystem, FlyWheelSubsystem flyWheel, 
  HopperOmniSubsystem hopperOmni, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new Timer(5000),
        new IntakeBalls(intake, 0.8),
        new AimWithVision(driveTrain, visionSubsystem, 0.0, 0.0, false),
        new ShootBall(flyWheel, hopperOmni, visionSubsystem)
      ),
      new StopFlyWheel(flyWheel)
    );
  }
}
