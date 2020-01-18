/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Counter;



/**
 * LIDAR-lite example
 * This gets the pulse width of the LIDAR sensor and converts it to a distance 
 * Inspired by https://github.com/GirlsOfSteelRobotics/Docs/wiki/LIDAR-Lite-Distance-Sensor
 */


public class Robot extends TimedRobot {
 
 private Counter m_LIDAR;


  @Override
  public void robotInit() {

 m_LIDAR = new Counter(0);
 //plug the lidar into DIO 0
  
 m_LIDAR.setMaxPeriod(1.00);
 //set the max period that can be measured
  
  m_LIDAR.setSemiPeriodMode(true);
 //Set the counter to period measurement
   
 m_LIDAR.reset();
  }
 
 final double off  = 0; 
//offset for sensor. test with tape measure

  @Override
  public void robotPeriodic() {
    double dist;
    double length;
    length = 0;
   if(m_LIDAR.get() < 1)
        dist = 0;
        
    else
      dist = (m_LIDAR.getPeriod()*1000000.0/10.0) - off;

    if(m_LIDAR.get() > 1)
      length = (m_LIDAR.getPeriod()*1000000.0/10.0) - off;
      length *= .393701;
 //convert to distance. sensor is high 10 us for every centimeter.
 //if( > 1) {
   // dist /= .393701;
    //SmartDashboard.putNumber("Inches", dist);
  //}
    SmartDashboard.putNumber("Inches", length);
    SmartDashboard.putNumber("Distance", dist);
    SmartDashboard.putNumber("Centimeter", dist);

 //put the distance on the dashboard
 
  }

}
