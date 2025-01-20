// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
/** Add your docs here. */
public class AutoFactory {
     

        public SendableChooser<Integer> m_blueStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Integer> m_redStartChooser = new SendableChooser<Integer>();


        public int finalChoice = 0;

        int ampChoice;
        int ampChoiceLast;
        int subwfrchoice;
        int subwfrchoicelast;
        int sourceChoice;
        int sourceChoiceLast;

        public int validStartChoice = 0;

        public int minsbwfrauto;
        public int maxsbwfrauto;
        public int minsourceauto;
        public int maxsourceauto;
        public int minampauto;
        public int maxampauto;


      

       PathFactory m_pf;

        public boolean validChoice;

        public AutoFactory(PathFactory pf) {
               m_pf=pf;

                minsbwfrauto = 1;
                m_blueStartChooser.setDefaultOption("Not Used", 0);
                m_blueStartChooser.addOption("W2-W1-W3", 1);
                m_blueStartChooser.addOption("W2-W3-W1", 2);
               

                maxsbwfrauto = 7;

                minsourceauto = 11;
                m_redStartChooser.setDefaultOption("Not Used", 10);
                m_redStartChooser.addOption("C4 Then C5", 11);
                m_redStartChooser.addOption("C5 Then C4", 12);

               

                SmartDashboard.putData("Blue Auto", m_blueStartChooser);
                SmartDashboard.putData("Red Auto", m_redStartChooser);
               
        }

        // This method is run by an EventLoop in RobotContainer
        public boolean checkChoiceChange() {

              boolean temp =false;
                return temp;
        }

        public int selectAndLoadPathFiles() {
                finalChoice = 0;
                validChoice = false;
                boolean validSubwfrChoice = subwfrchoice != 0;
                boolean validSourceChoice = sourceChoice != 10;
                boolean validAmpChoice = ampChoice != 20;

                if (validAmpChoice && !validSourceChoice && !validSubwfrChoice) {
                        m_pf.linkAmpPaths();
                        validChoice = true;
                        finalChoice = ampChoice;
                }

                if (validSourceChoice && !validAmpChoice && !validSubwfrChoice) {
                        m_pf.linkSourcePaths();
                        validChoice = true;
                        finalChoice = sourceChoice;
                }

                if (!validAmpChoice && !validSourceChoice && validSubwfrChoice) {
                      //  m_pf.linkRedPaths();
                        validChoice = true;
                        finalChoice = subwfrchoice;
                }

                SmartDashboard.putBoolean("Auto//Valid Auto Start Choice", validChoice);

                return finalChoice;
        }

        public Command finalCommand(int choice) {

                switch ((choice)) {

                        case 1:
                         
                        case 2:
                                

                        case 3:
                                

                        case 4:
                                

                        case 5:
                        
                        default:
                                return Commands.none();
                }
        }

        public Command getAutonomousCommand() {
                return finalCommand(finalChoice);
        }

}