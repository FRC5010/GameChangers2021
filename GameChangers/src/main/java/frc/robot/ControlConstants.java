/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.HIDType;

/**
 * Add your docs here.
 */
public class ControlConstants {
    static enum ButtonNums {
        NO_BUTTON, A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, LEFT_BUMPER, RIGHT_BUMPER, START_BUTTON, BACK_BUTTON,LEFT_STICK_BUTT, RIGHT_STICK_BUTT;
    }
    static enum AxisNums {
    LEFT_X, LEFT_Y, 
     L_TRIGGER, 
     R_TRIGGER, 
    RIGHT_X, RIGHT_Y
    }
    static enum POVDirs {
        UP, RIGHT, DOWN, LEFT 
    }
    // Driver
    public static int throttle = AxisNums.LEFT_Y.ordinal();
    public static int steer = AxisNums.RIGHT_X.ordinal();
    public static int steerY = AxisNums.RIGHT_Y.ordinal();
    //public static int winch1Axis = AxisNums.L_TRIGGER.ordinal();
    //public static int winch2Axis = AxisNums.R_TRIGGER.ordinal();

    public static int manualShootButton = ButtonNums.A_BUTTON.ordinal();
    public static int shooterAimButton = ButtonNums.B_BUTTON.ordinal();
    public static int triangleShoot = ButtonNums.RIGHT_BUMPER.ordinal();
    //public static int rotationControl = ButtonNums.X_BUTTON.ordinal();
    //public static int positionControl = ButtonNums.Y_BUTTON.ordinal();
    public static int startClimb = ButtonNums.LEFT_BUMPER.ordinal();
    //public static int spinDeploy = ButtonNums.RIGHT_BUMPER.ordinal();
    public static int calibrate = ButtonNums.START_BUTTON.ordinal(); 
    //public static int startClimb = ButtonNums.BACK_BUTTON.ordinal();
    public static int toggleDrive = ButtonNums.LEFT_STICK_BUTT.ordinal();
    public static int toggleLed = ButtonNums.RIGHT_STICK_BUTT.ordinal();
    
    public static int incThrottleFactor = POVDirs.UP.ordinal() * 90;
    public static int decThrottleFactor = POVDirs.DOWN.ordinal() * 90;
    public static int decSteerFactor = POVDirs.LEFT.ordinal() * 90;
    public static int incSteerFactor = POVDirs.RIGHT.ordinal() * 90;

    //Operator
    public static int operatorLeftY = AxisNums.LEFT_Y.ordinal(); // Implement
    public static int operatorRightY = AxisNums.RIGHT_Y.ordinal(); // Implement
    public static int outtakeAxis = AxisNums.L_TRIGGER.ordinal() ;
    public static int intakeAxis = AxisNums.R_TRIGGER.ordinal();

    public static int launchButton = ButtonNums.A_BUTTON.ordinal();
    public static int hoodDown = ButtonNums.B_BUTTON.ordinal(); 
    //public static int lowGoalShoot = ButtonNums.X_BUTTON.ordinal(); // not used
    public static int hoodUp = ButtonNums.Y_BUTTON.ordinal(); 
    public static int intakeUpButton = ButtonNums.LEFT_BUMPER.ordinal();
    public static int intakeDownButton = ButtonNums.RIGHT_BUMPER.ordinal();
    public static int startFlywheel = ButtonNums.LEFT_STICK_BUTT.ordinal();
    public static int stopFlywheel = ButtonNums.RIGHT_STICK_BUTT.ordinal();
    public static int overrideIntake = ButtonNums.START_BUTTON.ordinal();

    public static int incShooter = POVDirs.UP.ordinal() * 90;
    public static int decShooter = POVDirs.DOWN.ordinal() * 90;
    public static int spinnerOverrideButtonLow = POVDirs.RIGHT.ordinal() * 90;
    public static int spinnerOverrideButtonHigh = POVDirs.LEFT.ordinal() * 90;

    // Shuffleboard constants
    public static String SBTabDriverDisplay = "Driver Display";
    public static String SBTabVisionDisplay = "Vision Display";
    public static String SBTabDiagnostics = "Diagnostics";
    public static int shooterColumn = 0;
    public static int hoodColumn = 2;

    public static int driverColumn = 4;
    public static int autoColumn = 4;
    public static int intakeVisionColumn = 0;
    public static int shooterVisionColumn = 6;

    public static int autoNavButton = ButtonNums.X_BUTTON.ordinal();
    public static int driveTrainCurrentLimit = 38;
    
    public static boolean setupSingleDriver(Joystick operator){
        if(operator.getType() == HIDType.kUnknown){
            throttle = AxisNums.LEFT_Y.ordinal();
            steer = AxisNums.RIGHT_X.ordinal();
        
            startClimb = ButtonNums.LEFT_BUMPER.ordinal();
        
            launchButton = ButtonNums.A_BUTTON.ordinal();
            autoNavButton = ButtonNums.X_BUTTON.ordinal();
            return true;
        }
        return false;
    }
}
