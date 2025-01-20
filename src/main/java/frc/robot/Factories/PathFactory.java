// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** Add your docs here. */
public class PathFactory {

    private final SwerveSubsystem m_swerve;

    int ampChoice;
    int ampChoiceLast;

    int sourceChoice;
    int sourceChoiceLast;

    public boolean ampFilesOK;

    public boolean sourceFilesOK;

    public HashMap<String, PathPlannerPath> pathMaps = new HashMap<String, PathPlannerPath>();

    public PathFactory(SwerveSubsystem swerve) {
        m_swerve = swerve;
    }

    public enum bluepaths {

    }

    public boolean checkAmpFilesExist() {
        int valid = 0;
        for (bluepaths a : bluepaths.values()) {
            if (new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + a.name() + ".path").isFile())
                valid++;
            SmartDashboard.putNumber("Valid", valid);
        }
        return valid == bluepaths.values().length;
    }

    public boolean linkAmpPaths() {
        pathMaps.clear();
        for (bluepaths a : bluepaths.values()) {
            pathMaps.put(a.name(), getPath(a.name()));
        }
        return true;
    }

    public enum redpaths {

    }

    public boolean checkSourceFilesExist() {
        int valid = 0;
        for (redpaths a : redpaths.values()) {
            if (new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + a.toString() + ".path").isFile())
                valid++;
        }
        return valid == redpaths.values().length;
    }

    public void linkSourcePaths() {
        pathMaps.clear();
        for (redpaths s : redpaths.values()) {
            pathMaps.put(s.toString(), getPath(s.toString()));
        }
    }

    public PathPlannerPath getPath(String pathname) {
        PathPlannerPath p = null;
        try {
            p = PathPlannerPath.fromPathFile(pathname);
        } catch (Exception e) {
            // TODO: handle exception
        }
        return p;

    }

}