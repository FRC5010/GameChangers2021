// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;
import java.util.concurrent.ScheduledFuture;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.RamseteFollower;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSystem;

public class DetermineGalacticPath extends CommandBase {
  /** Creates a new DetermineGalacticPath. */
  private VisionSystem visionSystem;
  private IntakeSubsystem intakeSubsystem;

  private double aRed = -9.3;
  private double aBlue = 19.25;
  private double bRed = -20;
  private double bBlue = 14;

  String aRedPath = "paths/ARed.wpilib.json";
  String aBluePath = "paths/ABlue.wpilib.json";
  String bRedPath = "paths/PathBRed.wpilib.json";
  String bBluePath = "paths/PathBBlue.wpilib.json";
  Command aRedAuto;
  Command aBlueAuto;
  Command bRedAuto;
  Command bBlueAuto;

  public DetermineGalacticPath(VisionSystem visionSystem, IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.visionSystem = visionSystem;

    aRedAuto = Drive.getAutonomousCommand(aRedPath,true);
    aBlueAuto = Drive.getAutonomousCommand(aBluePath,true);
    bRedAuto = Drive.getAutonomousCommand(bRedPath,true);
    bBlueAuto = Drive.getAutonomousCommand(bBluePath,true);



    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double xAngle = visionSystem.getAngleX();
    double aRedError = Math.abs(xAngle - aRed);
    double aBlueError = Math.abs(xAngle - aBlue);
    double bRedError = Math.abs(xAngle - bRed);
    double bBlueError = Math.abs(xAngle - bBlue);

    Command pathToRun = new InstantCommand();
    if (aRedError < aBlueError && aRedError < bRedError && aRedError < bBlueError) {
      pathToRun = aRedAuto;
    } else if (aBlueError < aRedError && aBlueError < bRedError && aBlueError < bBlueError){
      pathToRun = aBlueAuto;
    } else if(bRedError < aRedError && bRedError < aBlueError && bRedError < bBlueError){
      pathToRun = bRedAuto;
    } else if(bBlueError < aRedError && bBlueError < aBlueError && bBlueError < bRedError){
      pathToRun = bBlueAuto;
    }
    SmartDashboard.putString("Selected GS Path", pathToRun.getName());
    CommandScheduler.getInstance().schedule(new ScriptedGalacticSearch(intakeSubsystem, pathToRun));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
