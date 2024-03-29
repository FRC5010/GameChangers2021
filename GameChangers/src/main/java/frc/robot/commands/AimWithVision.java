/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.VisionSystem;

public class AimWithVision extends CommandBase {
  
  // AimWithVision just turns the bot till it sees the vision target;

  DriveTrainMain drive;
  VisionSystem vision;
  Joystick driver;
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

  public AimWithVision(DriveTrainMain drive, VisionSystem vision, Joystick driver, double targetAngle) {
    this.drive = drive;
    this.vision = vision;
    this.driver = driver;
    this.targetAngle = targetAngle;
    this.p = 0.02;
    this.d = 0.001 ;
    addRequirements(drive);

    SmartDashboard.putNumber("Aim w/ vision p", p);
    SmartDashboard.putNumber("Aim w/ vision d", d);
  }

  public AimWithVision(DriveTrainMain drive, VisionSystem vision, double targetAngle, double driveSpeed, boolean timeOut) {
    this.drive = drive;
    this.vision = vision;
    this.driver = null;
    this.targetAngle = targetAngle;
    this.driveSpeed = driveSpeed;
    this.p = 0.009;
    this.d = 0.001;
    this.timeOut = timeOut;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = RobotController.getFPGATime();
    if (vision.isValidTarget()) {
      
      p = SmartDashboard.getNumber("Aim w/ vision p", p);
      d = SmartDashboard.getNumber("Aim w/ vision d", d);
      
      timeTargetFound = currentTime;

      lastError = error;
      error = vision.getAngleX() - targetAngle;
      SmartDashboard.putNumber("Vision Aim Error", error);
      double correction = error * p;
      SmartDashboard.putNumber("Correction", correction);
      //correction = Math.max(-d, Math.min(d, correction));
      //correction = Math.abs(correction);
      if (driver != null) {
        driveSpeed = drive.scaleInputs(-driver.getRawAxis(ControlConstants.throttle));
      }
      drive.arcadeDrive(driveSpeed, correction);
      lastTime = currentTime;
      SmartDashboard.putNumber(vision.getName() + "VisionError", error);
    }
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (driver == null || timeOut) {
      // If vision target not acquired and we've exceeded the acquisition timeout, don't return
      // Because, that could mean we're shooting in the wrong direction!
      if (!vision.isValidTarget() && currentTime - timeTargetFound > 2000) {
        vision.flashLight();
        return false;
      } else {
        return Math.abs(error) < angleTolerance;
      }
    } else {
      return false; 
    }
  }
}
