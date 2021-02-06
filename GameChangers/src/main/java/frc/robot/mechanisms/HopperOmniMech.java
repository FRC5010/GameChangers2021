/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.HopperOmniSubsystem;

/**
 * Add your docs here.
 */
public class HopperOmniMech {
    private CANSparkMax HOmniMotor;
    private Joystick driver;
    public HopperOmniSubsystem hopperOmniSubsystem;
    private CANSparkMax hopperMotor;
    public HopperOmniMech(Joystick driver){
        this.driver = driver;
        init(driver);
    }
    private void init(Joystick driver){
        //this.driver = driver;
        
        
        
    }
}
