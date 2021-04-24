/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HopperOmni;
import frc.robot.commands.auto.AutoTestPath;
import frc.robot.commands.auto.LeftShoot3RP;
import frc.robot.commands.auto.Shoot3RpShoot2;
import frc.robot.commands.auto.Shoot3Trench3;
import frc.robot.mechanisms.ClimbMech;
import frc.robot.mechanisms.Drive;
import frc.robot.mechanisms.FlyWheelMech;
import frc.robot.mechanisms.IntakeMech;
import frc.robot.subsystems.VisionLimeLight;
import frc.robot.subsystems.VisionLimeLightH;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Joystick driver;
  public JoystickButton autoNavButton;

  private VisionLimeLight shooterVision;

  private HopperOmni hOmni;
  private Drive drive;
  private Joystick operator;
  private FlyWheelMech flyWheelMech;
  private IntakeMech intakeMech;
  private VisionLimeLightH intakeVision;
  private ClimbMech climb;

  private SendableChooser<Command> command = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    shooterVision = new VisionLimeLight("limelight-shooter", 19.25, 14.562694, 90, ControlConstants.shooterVisionColumn);
    intakeVision = new VisionLimeLightH("limelight-intake", 24, -5, 6, ControlConstants.shooterVisionColumn);
    driver = new Joystick(0);
    operator = new Joystick(1);
    intakeMech = new IntakeMech(operator, driver);
    drive = new Drive(driver,shooterVision);
    flyWheelMech = new FlyWheelMech(driver, operator, shooterVision);
    climb = new ClimbMech(driver, operator,flyWheelMech);

    
    

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //autoNavButton = new JoystickButton(driver,  ControlConstants.autoNavButton);
    //autoNavButton.whenPressed(new DetermineGalacticPath(intakeVision, IntakeMech.getIntakeSubsystem()));
    command.setDefaultOption("test", new AutoTestPath(drive.getDriveTrainMain(), shooterVision, flyWheelMech.getFlyWheelSubsystem(), flyWheelMech.getHopperOmniSubsystem(), intakeMech.getIntakeSubsystem()));
    command.addOption("Right Trench", new Shoot3Trench3(drive.getDriveTrainMain(), shooterVision, flyWheelMech.getFlyWheelSubsystem(), flyWheelMech.getHopperOmniSubsystem(), intakeMech.getIntakeSubsystem()));
    command.addOption("Right RP", new Shoot3RpShoot2(drive.getDriveTrainMain(), shooterVision, flyWheelMech.getFlyWheelSubsystem(), flyWheelMech.getHopperOmniSubsystem(), intakeMech.getIntakeSubsystem()));
    command.addOption("Left RP", new LeftShoot3RP(drive.getDriveTrainMain(), shooterVision, flyWheelMech.getFlyWheelSubsystem(), flyWheelMech.getHopperOmniSubsystem(), intakeMech.getIntakeSubsystem()));

    Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay).getLayout("Auto", BuiltInLayouts.kList)
        .withPosition(ControlConstants.autoColumn, 0).withSize(3, 1).add("Choose an Auto Mode", command)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return command.getSelected();
  }
}
