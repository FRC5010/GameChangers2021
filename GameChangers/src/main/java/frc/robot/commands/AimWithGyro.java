// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlConstants;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.VisionSystem;

public class AimWithGyro extends CommandBase {
  DriveTrainMain drive;
  VisionSystem vision;
  double targetAngle, driveSpeed;

  double angleTolerance = 1;
  double error;
  double lastError;
  double timeTargetFound;
  double currentTime;
  double lastTime;
  double p;
  double d;
  boolean timeOut = true;
  Pose pose;
  double poseInit;
  /** Creates a new AimWithGyro. */

  public AimWithGyro(DriveTrainMain drive, VisionSystem vision, double targetAngle, double driveSpeed, boolean timeOut, Pose pose) {
    this.drive = drive;
    this.vision = vision;
    this.targetAngle = targetAngle;
    this.driveSpeed = driveSpeed;
    this.p = 0.015;
    this.d = 0.001;
    this.timeOut = timeOut;
    this.pose = pose;
    addRequirements(drive);
    SmartDashboard.putNumber("Aim w/ vision p", p);
    SmartDashboard.putNumber("Aim w/ vision d", d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.setLight(true);
    timeTargetFound = RobotController.getFPGATime();
    currentTime = RobotController.getFPGATime();
    error = angleTolerance;
    poseInit = pose.getHeading();
    targetAngle = poseInit - vision.getAngleX();
    SmartDashboard.putNumber("Gyro Aim Angle", targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.isValidTarget()) {
      
      p = SmartDashboard.getNumber("Aim w/ vision p", p);
      d = SmartDashboard.getNumber("Aim w/ vision d", d);
      
      timeTargetFound = currentTime;

      lastError = error;
      error = pose.getHeading() - targetAngle;
      SmartDashboard.putNumber("Vision Aim Error", error);
      double correction = error * p;
      SmartDashboard.putNumber("Correction", correction);
      //correction = Math.max(-d, Math.min(d, correction));
      //correction = Math.abs(correction);
      drive.arcadeDrive(DriveTrainMain.scaleInputs(-Drive.driver.getRawAxis(ControlConstants.throttle)), correction);
      lastTime = currentTime;
      SmartDashboard.putNumber(vision.getName() + "VisionError", error);
      if(Math.abs(error) < angleTolerance){
        poseInit = pose.getHeading();
        targetAngle = poseInit - vision.getAngleX();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timeOut) {
      // If vision target not acquired and we've exceeded the acquisition timeout, don't return
      // Because, that could mean we're shooting in the wrong direction!
      if (!vision.isValidTarget() && currentTime - timeTargetFound > 2000) {
        vision.flashLight();
        return true;
      } else {
        return false;
      }
    } else {
      return false; 
    }
  }
}
