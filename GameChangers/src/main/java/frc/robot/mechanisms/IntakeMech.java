// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class IntakeMech {
    private CANSparkMax m9;
    private CANSparkMax m11;
    private CANSparkMax m12;
    private Joystick operator;

    public IntakeMech(Joystick operator){
        this.operator = operator;
        m9 = new CANSparkMax(9, MotorType.kBrushless);
        m11 = new CANSparkMax(11, MotorType.kBrushless);
        m11.follow(m9);
        m12 = new CANSparkMax(12, MotorType.kBrushless);

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(m9, m11, m12, operator);
        intakeSubsystem.setDefaultCommand(new ToggleIntake(intakeSubsystem));

    }
}
