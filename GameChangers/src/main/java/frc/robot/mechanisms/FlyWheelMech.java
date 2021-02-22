/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import java.util.ResourceBundle.Control;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.ControlConstants;
import frc.robot.commands.HopperOmni;
import frc.robot.commands.ShootBall;
import frc.robot.commands.ToggleHoodDown;
import frc.robot.commands.ToggleHoodUp;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HopperOmniSubsystem;

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
    private JoystickButton launch;
    private POVButton baseUp;
    private POVButton baseDown;
    private CANPIDController m_pidController;

    private void configurebuttonBindings(){
        hoodUp = new JoystickButton(operator, ControlConstants.hoodUp);
        hoodDown = new JoystickButton(operator, ControlConstants.hoodDown);
        launch = new JoystickButton(operator, ControlConstants.launchButton);
        baseUp = new POVButton(driver, ControlConstants.incShooter);
        baseDown = new POVButton(driver, ControlConstants.decShooter);

        hoodUp.whileHeld(new ToggleHoodUp(flyWheelSubsystem));
        hoodDown.whileHeld(new ToggleHoodDown(flyWheelSubsystem));
        launch.whileHeld(new ShootBall(flyWheelSubsystem, hopperOmniSubsystem));

        baseUp.whenPressed(new InstantCommand(() -> ShooterConstants.baseSpeed += 10));
        baseDown.whenPressed(new InstantCommand(() -> ShooterConstants.baseSpeed -= 10));
    }

    public FlyWheelMech(Joystick driver, Joystick operator){
        this.driver = driver;
        this.operator = operator;
        m1 = new CANSparkMax(6,MotorType.kBrushless);
        m2 = new CANSparkMax(7,MotorType.kBrushless);
        m1.setInverted(true);
        m2.follow(m1,true);
        m1.setSmartCurrentLimit(40);

        hopperMotor = new CANSparkMax(10, MotorType.kBrushless);
        HOmniMotor = new CANSparkMax(5, MotorType.kBrushless);

        hood = new CANSparkMax(8,MotorType.kBrushless);

        m_pidController = m1.getPIDController();

        flyWheelSubsystem = new FlyWheelSubsystem(m1, hood, m_pidController);
        //flyWheelSubsystem.setDefaultCommand(new FlyWheelDefault(operator, flyWheelSubsystem));
        hopperOmniSubsystem = new HopperOmniSubsystem(HOmniMotor, hopperMotor);
        hopperOmniSubsystem.setDefaultCommand(new HopperOmni(hopperOmniSubsystem, operator));

        configurebuttonBindings();
    }
}
