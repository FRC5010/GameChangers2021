// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/** Add your docs here. */
public class ShooterConstants {
    public static double kSC, kVC, kAC, kS, kV, kA, kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxAccel;

    public static final double earthGravity = 9.81; // m/s^2
    public static final double ballRadius = 0.09; // m
    public static final double innerPortRadius = 0.16; // m
    public static final double innerPortHeight = 2.500; // meters, measured from floor to center
    public static double baseSpeed = 4257;
    //4500 scores at 7 ft with distancetoRPM at 0
    //4750 scores at 10 ft (about 100 per feet until we hit 10 feet)
    //4830 scores at 12 ft
    //4850 scores better at 14 ft
    //4870 scores at 16 ft
    //4890 scores at 17 ft
    //4950 scores at 18 ft
    //Around 5180 at 19 ft
    public static double distanceToRPM = 3.56;
    public static double getBaseSpeed() { return baseSpeed; }
    public static double getDistanceToRPM() { return distanceToRPM; }
    static {
        kSC = .329;
        kVC = 0.334;
        kAC = 0.0967;
        kS = kSC / 12;
        kV = kVC / 60 / 1 / (12 - kS);
        kA = kAC / 60 / 1 / (12 - kS);
        kP = 2.7/12;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 1.0 / 4600.0;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5800;
    }
}
