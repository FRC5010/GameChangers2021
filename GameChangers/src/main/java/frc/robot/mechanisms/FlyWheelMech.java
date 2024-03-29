/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.ControlConstants;
import frc.robot.commands.AimWithGyro;
import frc.robot.commands.CameraCalibrateShooter;
import frc.robot.commands.HopperOmni;
import frc.robot.commands.ManualShootBall;
import frc.robot.commands.ShootBall;
import frc.robot.commands.StartFlyWheel;
import frc.robot.commands.StopFlyWheel;
import frc.robot.commands.ToggleHoodDown;
import frc.robot.commands.ToggleHoodUp;
import frc.robot.commands.TriangleHood;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HopperOmniSubsystem;
import frc.robot.subsystems.VisionSystem;

/**
 * Add your docs here.
 */
public class FlyWheelMech {
    public CANSparkMax m1;
    public CANSparkMax m2;
    public CANSparkMax hood;
    public CANSparkMax hopperMotor;
    public CANSparkMax HOmniMotor;

    public FlyWheelSubsystem flyWheelSubsystem;
    public HopperOmniSubsystem hopperOmniSubsystem;
    private Joystick driver;
    private Joystick operator;
    private JoystickButton hoodUp;
    private JoystickButton hoodDown;
    private JoystickButton triangleHoodAim;
    private JoystickButton launch;
    private JoystickButton manualLaunch;
    private JoystickButton calibrate;
    private JoystickButton lightToggle;
    private JoystickButton stopFlyWheel;
    private JoystickButton startFlywheel;
    private POVButton baseUp;
    private POVButton baseDown;
    private CANPIDController m_pidController;
    private VisionSystem vision;

    private void configurebuttonBindings(){
        hoodUp = new JoystickButton(operator, ControlConstants.hoodUp);
        hoodDown = new JoystickButton(operator, ControlConstants.hoodDown);
        triangleHoodAim = new JoystickButton(driver, ControlConstants.triangleShoot);
        baseUp = new POVButton(operator, ControlConstants.incShooter);
        baseDown = new POVButton(operator, ControlConstants.decShooter);
        manualLaunch = new JoystickButton(driver, ControlConstants.manualShootButton);

        launch = new JoystickButton(operator, ControlConstants.launchButton);
        calibrate = new JoystickButton(driver, ControlConstants.calibrate);
        lightToggle = new JoystickButton(driver, ControlConstants.toggleLed);

        stopFlyWheel = new JoystickButton(operator, ControlConstants.stopFlywheel);
        startFlywheel = new JoystickButton(operator, ControlConstants.startFlywheel);
        

        hoodUp.whenPressed(new ToggleHoodUp(flyWheelSubsystem));
        hoodDown.whenPressed(new ToggleHoodDown(flyWheelSubsystem));
        triangleHoodAim.whileHeld(new TriangleHood(flyWheelSubsystem));
        manualLaunch.whileHeld(new ManualShootBall(flyWheelSubsystem, hopperOmniSubsystem));
        launch.whileHeld(new ShootBall(flyWheelSubsystem, hopperOmniSubsystem, vision));
        /*launch.whileHeld(new ParallelRaceGroup(
            new AimWithGyro(Drive.driveTrain, vision, 0.0, 0.0, false, Drive.robotPose),
            new ShootBall(flyWheelSubsystem, hopperOmniSubsystem, vision)
        ));
        */
        baseUp.whenPressed(new InstantCommand(() -> ShooterConstants.baseSpeed += 10));
        baseDown.whenPressed(new InstantCommand(() -> ShooterConstants.baseSpeed -= 10));
        calibrate.whenPressed(new CameraCalibrateShooter(vision));
        lightToggle.whenPressed(new InstantCommand(() -> vision.setLight(!vision.isLightOn())));

        stopFlyWheel.whenPressed(new StopFlyWheel(flyWheelSubsystem));
        startFlywheel.whenPressed(new StartFlyWheel(flyWheelSubsystem));
    }

    public FlyWheelMech(Joystick driver, Joystick operator, VisionSystem vision){
        this.driver = driver;
        this.operator = operator;
        this.vision = vision;

        m1 = new CANSparkMax(6,MotorType.kBrushless);
        m2 = new CANSparkMax(7,MotorType.kBrushless);
        m1.restoreFactoryDefaults();
        m2.restoreFactoryDefaults();

        m1.setInverted(true);
        m2.follow(m1,true);
        m1.setSmartCurrentLimit(40);

        hopperMotor = new CANSparkMax(10, MotorType.kBrushless);
        HOmniMotor = new CANSparkMax(5, MotorType.kBrushless);

        hopperMotor.restoreFactoryDefaults();
        HOmniMotor.restoreFactoryDefaults();
        
        hopperMotor.setSmartCurrentLimit(40);
        HOmniMotor.setSmartCurrentLimit(40);


        hood = new CANSparkMax(8,MotorType.kBrushless);
        hood.setSmartCurrentLimit(40);

        m_pidController = m1.getPIDController();

        flyWheelSubsystem = new FlyWheelSubsystem(m1, hood, m_pidController,vision);
        hopperOmniSubsystem = new HopperOmniSubsystem(HOmniMotor, hopperMotor);
        hopperOmniSubsystem.setDefaultCommand(new HopperOmni(hopperOmniSubsystem, operator));

        configurebuttonBindings();
    }

    public FlyWheelSubsystem getFlyWheelSubsystem(){
        return flyWheelSubsystem;
    }

    public HopperOmniSubsystem getHopperOmniSubsystem(){
        return hopperOmniSubsystem;
    }
}
