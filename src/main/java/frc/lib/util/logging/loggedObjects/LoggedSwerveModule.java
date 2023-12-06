// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.loggedPrimitives.LoggedDoubleArray;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class LoggedSwerveModule extends LoggedObject<SwerveModule[]> {

    

    public LoggedSwerveModule(String name, LoggedContainer subsystemLogger, SwerveModule[] object, String logType,
            String tabName) {
        super(name, subsystemLogger, object, logType, tabName);
    }

    public LoggedSwerveModule(String name, LoggedContainer subsystemLogger, SwerveModule[] object, String logType,
            Boolean SeparateTab) {
        super(name, subsystemLogger, object, logType, SeparateTab);
    }

    public LoggedSwerveModule(String name, LoggedContainer subsystemLogger, SwerveModule[] object, String logType) {
        super(name, subsystemLogger, object, logType);
    }

    @Override
    protected void initializeShuffleboard() {

       
        ShuffleboardLayout layout0 = getTab().getLayout("Module:0", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object[0].getCanCoderWithOffset().getRotations(), layout0);
        addDoubleToShuffleboard("Angle", () -> object[0].getState().angle.getRotations(), layout0);
        addDoubleToShuffleboard("Speed", () -> object[0].getState().speedMetersPerSecond, layout0);
        addDoubleToShuffleboard("TotalDistance", () -> object[0].getPosition().distanceMeters, layout0);
        addDoubleToShuffleboard("TotalRotations", () -> object[0].getRotations(), layout0);
       
        ShuffleboardLayout layout1 = getTab().getLayout("Module:1", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object[1].getCanCoderWithOffset().getRotations(), layout1);
        addDoubleToShuffleboard("Angle", () -> object[1].getState().angle.getRotations(), layout1);
        addDoubleToShuffleboard("Speed", () -> object[1].getState().speedMetersPerSecond, layout1);
        addDoubleToShuffleboard("TotalDistance", () -> object[1].getPosition().distanceMeters, layout1);
        addDoubleToShuffleboard("TotalRotations", () -> object[1].getRotations(), layout1);

        ShuffleboardLayout layout2 = getTab().getLayout("Module:2", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object[2].getCanCoderWithOffset().getRotations(), layout2);
        addDoubleToShuffleboard("Angle", () -> object[2].getState().angle.getRotations(), layout2);
        addDoubleToShuffleboard("Speed", () -> object[2].getState().speedMetersPerSecond, layout2);
        addDoubleToShuffleboard("TotalDistance", () -> object[2].getPosition().distanceMeters, layout2);
        addDoubleToShuffleboard("TotalRotations", () -> object[2].getRotations(),layout2);

        ShuffleboardLayout layout3 = getTab().getLayout("Module:3", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object[3].getCanCoderWithOffset().getRotations(), layout3);
        addDoubleToShuffleboard("Angle", () -> object[3].getState().angle.getRotations(), layout3);
        addDoubleToShuffleboard("Speed", () -> object[3].getState().speedMetersPerSecond, layout3);
        addDoubleToShuffleboard("TotalDistance", () -> object[3].getPosition().distanceMeters, layout3);
        addDoubleToShuffleboard("TotalRotations", () -> object[3].getRotations(), layout3);


        //FL, FR, BL, BR
        ShuffleboardLayout trueStatesLayout = getTab().getLayout("TrueModuleStates", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleArrayToShuffleboard(name, () -> 
            new double[]{
                object[0].getState().angle.getDegrees(), object[0].getState().speedMetersPerSecond,
                object[1].getState().angle.getDegrees(),  object[1].getState().speedMetersPerSecond,
                object[2].getState().angle.getDegrees(), object[2].getState().speedMetersPerSecond,
                object[3].getState().angle.getDegrees(), object[3].getState().speedMetersPerSecond
            
        }, trueStatesLayout);

        // System.out.println(object[0].getState());


        ShuffleboardLayout desiredStatesLayout = getTab().getLayout("DesiredModuleStates", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleArrayToShuffleboard(name, () -> 
            new double[]{
                object[0].getDesiredState().angle.getDegrees(), object[0].getDesiredState().speedMetersPerSecond,
                object[1].getDesiredState().angle.getDegrees(),  object[1].getDesiredState().speedMetersPerSecond,
                object[2].getDesiredState().angle.getDegrees(), object[2].getDesiredState().speedMetersPerSecond,
                object[3].getDesiredState().angle.getDegrees(), object[3].getDesiredState().speedMetersPerSecond
        }, desiredStatesLayout);   

        



        //FL, FR, BL, BR
        add(new LoggedDoubleArray(this, () -> 
            new double[]{
                object[0].getState().angle.getDegrees(), object[0].getState().speedMetersPerSecond,
                object[1].getState().angle.getDegrees(),  object[1].getState().speedMetersPerSecond,
                object[2].getState().angle.getDegrees(), object[2].getState().speedMetersPerSecond,
                object[3].getState().angle.getDegrees(), object[3].getState().speedMetersPerSecond
            
        }, NetworkTableInstance.getDefault().getTable(getPrefix()).getSubTable(name).getDoubleArrayTopic("TrueStates").getGenericEntry()));

        add(new LoggedDoubleArray(this, () -> 
            new double[]{
                object[0].getDesiredState().angle.getDegrees(), object[0].getDesiredState().speedMetersPerSecond,
                object[1].getDesiredState().angle.getDegrees(),  object[1].getDesiredState().speedMetersPerSecond,
                object[2].getDesiredState().angle.getDegrees(), object[2].getDesiredState().speedMetersPerSecond,
                object[3].getDesiredState().angle.getDegrees(), object[3].getDesiredState().speedMetersPerSecond
            
        }, NetworkTableInstance.getDefault().getTable(getPrefix()).getSubTable(name).getDoubleArrayTopic("DesiredStates").getGenericEntry()));

    }

    @Override
    protected void initializeDataLog() {

        addDoubleToOnboardLog("Module:0/CanCoder", () -> object[0].getCanCoder().getDegrees());
        addDoubleToOnboardLog("Module:0/TotalDistance", () -> object[0].getPosition().distanceMeters);
        
        addDoubleToOnboardLog("Module:1/CanCoder", () -> object[1].getCanCoder().getDegrees());
        addDoubleToOnboardLog("Module:1/TotalDistance", () -> object[1].getPosition().distanceMeters);
        
        addDoubleToOnboardLog("Module:2/CanCoder", () -> object[2].getCanCoder().getDegrees());
        addDoubleToOnboardLog("Module:2/TotalDistance", () -> object[2].getPosition().distanceMeters);

        addDoubleToOnboardLog("Module:3/CanCoder", () -> object[3].getCanCoder().getDegrees());
        addDoubleToOnboardLog("Module:3/TotalDistance", () -> object[3].getPosition().distanceMeters);

        addDoubleArrayToOnboardLog("TrueStates", () -> 
        new double[]{
            object[0].getState().angle.getDegrees(), object[0].getState().speedMetersPerSecond,
                object[1].getState().angle.getDegrees(),  object[1].getState().speedMetersPerSecond,
                object[2].getState().angle.getDegrees(), object[2].getState().speedMetersPerSecond,
                object[3].getState().angle.getDegrees(), object[3].getState().speedMetersPerSecond
            }
        );

        addDoubleArrayToOnboardLog("DesiredStates", () -> 
        new double[]{
            object[0].getDesiredState().angle.getDegrees(), object[0].getDesiredState().speedMetersPerSecond,
                object[1].getDesiredState().angle.getDegrees(),  object[1].getDesiredState().speedMetersPerSecond,
                object[2].getDesiredState().angle.getDegrees(), object[2].getDesiredState().speedMetersPerSecond,
                object[3].getDesiredState().angle.getDegrees(), object[3].getDesiredState().speedMetersPerSecond
            }
        );
    }

}
