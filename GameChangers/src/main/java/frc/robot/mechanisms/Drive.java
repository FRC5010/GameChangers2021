/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import java.io.IOException;
import java.nio.file.Path;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.ControlConstants;
import frc.robot.Robot;
import frc.robot.commands.AimWithGyro;
import frc.robot.commands.AimWithVision;
import frc.robot.commands.RamseteFollower;
import frc.robot.commands.SwitchDriveDirection;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.VisionSystem;

/**
 * Add your docs here.
 */
public class Drive {
  public static DriveTrainMain driveTrain;
  private static VisionSystem shooterCam;

  public static Joystick driver;
  public static CANSparkMax lDrive1;
  public static CANSparkMax lDrive2;

  public static CANSparkMax rDrive1;
  public static CANSparkMax rDrive2;

  public static CANEncoder lEncoder;
  public static CANEncoder rEncoder;

  public static Pose robotPose;

  public JoystickButton intakeAimButton;
  public JoystickButton shooterAimButton;

  public POVButton incThrottleFactor;
  public POVButton decThrottleFactor;
  public POVButton incSteerFactor;
  public POVButton decSteerFactor;

  public JoystickButton switchDirection;

  public JoystickButton intakeDriveButton;
  public JoystickButton autoNavButton;

  public Drive(Joystick driver, VisionSystem shooterVision) {
    init(driver, shooterVision);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    shooterAimButton = new JoystickButton(driver, ControlConstants.shooterAimButton);
    shooterAimButton.whileHeld(new AimWithGyro(driveTrain, shooterCam, 0, 0, true, robotPose));

    incThrottleFactor = new POVButton(driver, ControlConstants.incThrottleFactor);
    incThrottleFactor.whenPressed(new InstantCommand(() -> DriveConstants.throttleFactor = Math.min(1,
        DriveConstants.throttleFactor + DriveConstants.drivingAdjustment)));

    decThrottleFactor = new POVButton(driver, ControlConstants.decThrottleFactor);
    decThrottleFactor.whenPressed(new InstantCommand(() -> DriveConstants.throttleFactor = Math.max(0,
        DriveConstants.throttleFactor - DriveConstants.drivingAdjustment)));

    incSteerFactor = new POVButton(driver, ControlConstants.incSteerFactor);
    incSteerFactor.whenPressed(new InstantCommand(
        () -> DriveConstants.steerFactor = Math.min(1, DriveConstants.steerFactor + DriveConstants.drivingAdjustment)));

    decSteerFactor = new POVButton(driver, ControlConstants.decSteerFactor);
    decSteerFactor.whenPressed(new InstantCommand(
        () -> DriveConstants.steerFactor = Math.max(0, DriveConstants.steerFactor - DriveConstants.drivingAdjustment)));

    switchDirection = new JoystickButton(driver, ControlConstants.toggleDrive);
    switchDirection.whenPressed(new SwitchDriveDirection(driver));
  }

  public static void setCurrentLimits(int currentLimit) {
    if (RobotBase.isReal()) {
      lDrive1.setSmartCurrentLimit(currentLimit);
      lDrive2.setSmartCurrentLimit(currentLimit);
      rDrive1.setSmartCurrentLimit(currentLimit);
      rDrive2.setSmartCurrentLimit(currentLimit);
    }
  }

  public void init(Joystick driver, VisionSystem shooterVision) {
    this.driver = driver;
    // Neos HAVE to be in brushless
    lDrive1 = new CANSparkMax(1, MotorType.kBrushless);
    lDrive2 = new CANSparkMax(2, MotorType.kBrushless);

    rDrive1 = new CANSparkMax(3, MotorType.kBrushless);
    rDrive2 = new CANSparkMax(4, MotorType.kBrushless);

    lDrive1.restoreFactoryDefaults();
    lDrive2.restoreFactoryDefaults();
    rDrive1.restoreFactoryDefaults();
    rDrive2.restoreFactoryDefaults();

    lDrive1.setInverted(false);
    lDrive2.follow(lDrive1, false);
    rDrive1.setInverted(true);
    rDrive2.follow(rDrive1, false);
    rDrive2.setInverted(false);

    lEncoder = lDrive1.getEncoder();
    rEncoder = rDrive1.getEncoder();

    setCurrentLimits(ControlConstants.driveTrainCurrentLimit);

    // lEncoder.setPositionConversionFactor(DriveConstants.distancePerPulse);
    // rEncoder.setPositionConversionFactor(-DriveConstants.distancePerPulse);

    // lEncoder.setVelocityConversionFactor(DriveConstants.distancePerPulse);
    // rEncoder.setVelocityConversionFactor(-DriveConstants.distancePerPulse);

    robotPose = new Pose(lEncoder, rEncoder);
    robotPose.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    shooterCam = shooterVision;
    driveTrain = new DriveTrainMain(lDrive1, rDrive1, driver, robotPose);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   */

  public static Command getAutonomousCommand(String path, boolean reset) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    String trajectoryJSON = path;
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    RamseteCommand ramseteCommand = new RamseteFollower(trajectory, reset);

    Command result = ramseteCommand;

    // Run path following command, then stop at the end.
    return result;
  }

  public DriveTrainMain getDriveTrainMain() {
    return driveTrain;
  }
}
