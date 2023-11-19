// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.regressions;

import frc.lib.team254.util.PolynomialRegression;

/** Add your docs here. */
public class ShooterHoodRegression {

    // TODO find these
    
    /*
     * In the form [x, y] where x is measured in meters from the target and y is the angle of the hood from 0.
     * 
     * [x (meters), y (degrees)]
     * 
     */
    private static double[][] hoodValues = {
            { 0, 0 },
            { 0, 0 },
       
    };
  

    public static PolynomialRegression hoodRegression = new PolynomialRegression(hoodValues, 1);

}
