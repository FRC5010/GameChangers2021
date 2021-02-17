// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControlConstants;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeUp;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class IntakeMech {
    private CANSparkMax m9;
    private CANSparkMax m11;
    private CANSparkMax intakeMotor;
    private Joystick operator;
    private JoystickButton intakeUp;
    private JoystickButton intakeDown;

    public IntakeMech(Joystick operator){
        this.operator = operator;
        m9 = new CANSparkMax(9, MotorType.kBrushless);
        m11 = new CANSparkMax(11, MotorType.kBrushless);
        m11.follow(m9,true);
        m9.setInverted(true);
        intakeMotor = new CANSparkMax(12, MotorType.kBrushless);
        intakeMotor.setInverted(true);

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(m9, intakeMotor);

        intakeSubsystem.setDefaultCommand(new IntakeBalls(intakeSubsystem, operator));

        intakeUp = new JoystickButton(operator, ControlConstants.intakeUpButton);
        intakeDown = new JoystickButton(operator, ControlConstants.intakeDownButton);

        intakeUp.whileHeld(new IntakeUp(intakeSubsystem));
        intakeDown.whileHeld(new IntakeDown(intakeSubsystem));

        
    }
}