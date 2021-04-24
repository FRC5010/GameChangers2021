// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControlConstants;
import frc.robot.commands.SepArmClimb;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;

/** Add your docs here. */
public class ClimbMech {
    private CANSparkMax armMotor1;
    private CANSparkMax armMotor2;

    private Joystick driver;
    private Joystick operator;

    private JoystickButton startClimb;

    private ClimbSubsystem climbSubsystem;

    public ClimbMech(Joystick driver, Joystick operator,FlyWheelMech flyWheelMech){
        armMotor1 = new CANSparkMax(13, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(14, MotorType.kBrushless);
        armMotor2.restoreFactoryDefaults();

        armMotor1.setInverted(true);
        armMotor2.setInverted(false);

        this.driver = driver;
        this.operator = operator;

        climbSubsystem = new ClimbSubsystem(armMotor1,armMotor2);


        startClimb = new JoystickButton(driver, ControlConstants.startClimb);
        startClimb.whileHeld(new SepArmClimb(operator, climbSubsystem,flyWheelMech.getHopperOmniSubsystem()));

    
    }
}
