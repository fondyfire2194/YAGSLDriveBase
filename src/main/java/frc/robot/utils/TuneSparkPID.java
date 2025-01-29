// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class TuneSparkPID {

    private final String m_name;

    private SparkMax m_motor;
    private SparkMaxConfig config = new SparkMaxConfig();
    private int tst;
    private boolean change;
    String temp;
    String tune = "Tune/";
    boolean changeP;
    boolean changeI = false;
    boolean changeD = false;

    REVLibError error;

    public TuneSparkPID(String name, SparkMax motor) {
        m_name = name;
        m_motor = motor;

        temp = m_name.concat(tune);
        SmartDashboard.putNumber(temp + " Kp", 0);
        SmartDashboard.putNumber(temp + " Ki", 0);
        SmartDashboard.putNumber(temp + " Kd", 0);

        SmartDashboard.putBoolean(temp + "Change", change);

        changeP = true;
        changeI = true;
        changeD = true;
    }

    public void tune() {

        SmartDashboard.putNumber(temp + " TST", tst);
        SmartDashboard.putString("temp", temp);

        double kp = SmartDashboard.getNumber(temp + " Kp", 0);
        double ki = SmartDashboard.getNumber(temp + " Ki", 0);
        double kd = SmartDashboard.getNumber(temp + " Kd", 0);

        double existKp = 0;
        double existKi = 0;
        double existKd = 0;

        if (changeP || changeI || changeD) {

            existKp = m_motor.configAccessor.closedLoop.getP();
            existKi = m_motor.configAccessor.closedLoop.getI();
            existKd = m_motor.configAccessor.closedLoop.getD();

            SmartDashboard.putString("TEMP", temp);

            SmartDashboard.putNumber(temp + "EXISTKP", existKp);
            SmartDashboard.putNumber(temp + "EXISTKI", existKi);
            SmartDashboard.putNumber(temp + "EXISTKD", existKd);

        }

        changeP = round5dp(existKp) != kp;
        changeD = round5dp(existKd) != kd;
        changeI = round5dp(existKi) != ki;

        if (SmartDashboard.getBoolean(temp + "Change", change) && (changeP || changeI || changeD)) {

            tst++;

            config.closedLoop
                    .p(kp)
                    .i(ki)
                    .d(kd);
            error = m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        }
    }

    private double round5dp(double val) {
        return Math.round(val * 100000) / val;

    }

}
