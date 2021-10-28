/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry; 
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
 
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private final XboxController con = new XboxController(20);
 
  private WPI_VictorSPX victorCIMright = new WPI_VictorSPX(6);
  private WPI_VictorSPX victorCIMleft = new WPI_VictorSPX(1);
  private WPI_VictorSPX victorRLrise = new WPI_VictorSPX(9);
  
  private WPI_VictorSPX victorshootlow = new WPI_VictorSPX(8);
  private WPI_VictorSPX victorshoothigh = new WPI_VictorSPX(4);
  
  private WPI_VictorSPX victorgetin = new WPI_VictorSPX(7);
 
  private TalonSRX _talon = new WPI_TalonSRX(10);
 
  private WPI_VictorSPX victorDriveLeft1 = new WPI_VictorSPX(3);
  private WPI_VictorSPX victorDriveLeft2 = new WPI_VictorSPX(2);
  private SpeedControllerGroup victorDriveLeft = new SpeedControllerGroup(victorDriveLeft2,victorDriveLeft1);
  private WPI_VictorSPX victorDriveRight1 = new WPI_VictorSPX(5);
  private WPI_VictorSPX victorDriveRight2 = new WPI_VictorSPX(0);
  private SpeedControllerGroup victorDriveRight = new SpeedControllerGroup(victorDriveRight2,victorDriveRight1);
  
  DifferentialDrive differentialdrive = new DifferentialDrive(victorDriveLeft, victorDriveRight);
  private AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private Encoder leftencoder = new Encoder(0,1,false);
  private Encoder rightencoder = new Encoder(2,3,true);
 
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    new Rotation2d(), new Pose2d(0, 0, new Rotation2d()));
  private Pose2d goal_1 =new Pose2d(1,0,new Rotation2d(0));
  private Pose2d goal_2 =new Pose2d(1,1,new Rotation2d(0));
  private Pose2d goal_3 =new Pose2d(0,1,new Rotation2d(0));
  private Pose2d goal_4 =new Pose2d(0,0,new Rotation2d(0));
  // private final DifferentialDriveKinematics m_kinematics =
  // new DifferentialDriveKinematics(0.655);

  double present = 10000.0;
  boolean pressA = false;
  boolean pressB = false;
  boolean pressX = false;
  boolean pressY = false;
  boolean pressPOV0 = false;
  boolean pressPOV270 = false;
  boolean pressdeliver = false;
  double  [] speed  ={0.4,0.6,0.8,1.0};
  int i = 0;
  int level = 0;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    _talon.set(ControlMode.Position, 0);//啟動時先轉回0
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
 
    _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30);
    _talon.setSensorPhase(true);
    _talon.setInverted(false);
    _talon.configNominalOutputForward(0,30);
    _talon.configNominalOutputReverse(0,30);
    _talon.configPeakOutputForward(1,30);
    _talon.configPeakOutputReverse(-1,30);
 
    _talon.configAllowableClosedloopError(0,0,30);
 
    _talon.config_kF(0,0,30);
    _talon.config_kP(0,0.15,30);
    _talon.config_kI(0,0,30);
    _talon.config_kD(0,1,30);
 
    int absolutePosition = _talon.getSensorCollection().getPulseWidthPosition();
    absolutePosition &= 0xFFF;
    if (true) { absolutePosition *= -1; }
    //if (false) { absolutePosition *= -1; }
    
    /*Set the quadrature (relative) sensor to match absolute */
    _talon.setSelectedSensorPosition(absolutePosition,0,30);
 
 
  }
 
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }
 
  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    leftencoder.setDistancePerPulse(0.1524*3.14/600);
    rightencoder.setDistancePerPulse(0.1524*3.14/600);
 
    ahrs.reset();
    leftencoder.reset();
    rightencoder.reset();
  }
 
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  
    var gyro = Rotation2d.fromDegrees(-ahrs.getAngle());
 
  double cur_deg = ahrs.getAngle()%360; // get the current robot rotation and confine it between [0 ~ 360)
   double target = -90; //( Math.atan(goal_1.getTranslation().getY()/goal_1.getTranslation().getX())*-180/Math.PI)%360+8;// set the target degree
  //target = (-target) % 360;
  double delta_deg = target - cur_deg;

  if (delta_deg >= 180) delta_deg -= 360;
  else if (delta_deg < -180) delta_deg += 360; // get the delta degree and confine it to [-180, 180)
  m_odometry.update(gyro, leftencoder.getDistance(), rightencoder.getDistance());
  System.out.println(ahrs.getAngle());
  switch(level){

  case 0:
  Translation2d delta_2 = goal_1.minus(m_odometry.getPoseMeters()).getTranslation();
  m_odometry.update(gyro, leftencoder.getDistance(), leftencoder.getDistance());
  if(delta_2.getNorm()<=0.05){
    differentialdrive.arcadeDrive(0,0);
    present=delta_2.getNorm();
    level++;
    }
  else{
    differentialdrive.arcadeDrive(0.55,0);
  }
  break;
 
  case 1:
  if(delta_deg>2) {
  differentialdrive.arcadeDrive(0,0.5);
}

  else if (delta_deg<-2){
  differentialdrive.arcadeDrive(0,-0.5);
}
else{
  differentialdrive.arcadeDrive(0,0);
  level++;
}
break;
  
case 2:
  
  Translation2d delta_3 = goal_2.minus(m_odometry.getPoseMeters()).getTranslation();
  m_odometry.update(gyro, leftencoder.getDistance(), leftencoder.getDistance());
  if(delta_3.getNorm()<=0.05){
    differentialdrive.arcadeDrive(0,0);
    level++;
  }
  
  else{
    differentialdrive.arcadeDrive(0.5,0);
  }
  break;
 

case 3:
  if(delta_deg>2) {
    differentialdrive.arcadeDrive(0,0.45);
  }

  else if (delta_deg<-2){
    differentialdrive.arcadeDrive(0,-0.45);
  }
  else{
    differentialdrive.arcadeDrive(0,0);
    level++;
  }
  break;
case 4:
  Translation2d delta_4 = goal_3.minus(m_odometry.getPoseMeters()).getTranslation();
  m_odometry.update(gyro, leftencoder.getDistance(), leftencoder.getDistance()); 
  if(delta_4.getNorm()<=0.05){
    differentialdrive.arcadeDrive(0,0);
    level++;
  }
  
  else{
    differentialdrive.arcadeDrive(0.5,0);
  }
  break;

case 5:
if(delta_deg>2) {
  differentialdrive.arcadeDrive(0,0.45);
}

else if (delta_deg<-2){
  differentialdrive.arcadeDrive(0,-0.45);
}
else{
  differentialdrive.arcadeDrive(0,0);
  level++;
}
break;

case 6:
Translation2d delta_5 = goal_4.minus(m_odometry.getPoseMeters()).getTranslation();
m_odometry.update(gyro, leftencoder.getDistance(), leftencoder.getDistance());
if(delta_5.getNorm()<=0.05){
  differentialdrive.arcadeDrive(0,0);
  level++;
}

else{
  differentialdrive.arcadeDrive(0.5,0);
}
break;
}
System.out.println(m_odometry.getPoseMeters().getTranslation());
System.out.println(level);
}
 
  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
  }
 
  /** 
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
///////////////////rise_up/////RL////////////////
  if(con.getYButtonPressed()){
    if(pressY==false){
      victorRLrise.set(0.2);
      pressY=true;
    }
    else if(pressY==true){
      victorRLrise.set(0);
      pressY=false;
    }  
  }
//////////////////rise_down///RL/////////////////
  if (con.getBButtonPressed()){
    if(pressB==false){
      victorRLrise.set(-0.2);
      pressB=true;
    }
    else if(pressB==true){
      victorRLrise.set(0);
      pressB=false;
    }  
  }
  ///////////////////CIM/////////////////
  if(con.getPOV()==270){
    //System.out.println("1-1");
    victorCIMright.set(0.55);
    victorCIMleft.set(-0.45);
  }
 
  else if(con.getPOV()==0){
    //System.out.println("2-1");
    victorCIMright.set(-0.45);
    victorCIMleft.set(0.45);
  }
  else{
    //System.out.println("2-2");
    victorCIMright.set(0);
    victorCIMleft.set(0);
  }
 
  
//////////////////////shoot////////////////////////////

  if(Math.abs(con.getTriggerAxis(Hand.kRight))>0.1){
    if(con.getTriggerAxis(Hand.kRight)>0){
      victorshoothigh.set(0.8);
      victorshootlow.set(1);
    }  
  }
  else{
    victorshoothigh.set(0);
    victorshootlow.set(0);
  } 
/////////////////deliver/////////////
    double pos = _talon.getSelectedSensorPosition(0);
    if(Math.abs(con.getTriggerAxis(Hand.kLeft))>0.1){
      if(con.getTriggerAxis(Hand.kLeft)>0){ 
        pressdeliver = false; 
        if(pressdeliver == false){
          pos  += 680000;
          _talon.set(ControlMode.Position, pos);
 
          //_talon.set(ControlMode.PercentOutput, 0);
          _talon.setSelectedSensorPosition(0);
        }
      }
    }
    if(con.getPOV()==90){
      pressdeliver = true;
      if(pressdeliver == true){
        pos -= 680000;
        _talon.set(ControlMode.Position, pos);
        _talon.setSelectedSensorPosition(0);
      }
    }
   
 
    // if(con.getPOV()==90){
    //   pressA = true;
    //   _talon.set(ControlMode.Position, 700000);
    //   }
    //   int pos = _talon.getSelectedSensorPosition(0);
    //   System.out.println(pos);
    //   if (Math.abs(pos - 700000) < 50000) {
    //   pressA = false;
    //   }
    //   if (!pressA) {
    //   _talon.setSelectedSensorPosition(0);
    //   _talon.set(ControlMode.PercentOutput, 0);
    //   }
 
  //if(Math.abs(con.getTriggerAxis(Hand.kLeft))>0.1){
    //   if(con.getTriggerAxis(Hand.kLeft)<0){
    //     _talon.set(ControlMode.Position, 70000);
    //     pressdeliver = true ;
    //   if(Math.abs(pos - 700000) < 50000) {
    //     pressdeliver = false;
    //   }
    //   }
    // //} 
      
    // if(con.getPOV()==90){
    //   _talon.set(ControlMode.Position, -70000);
    //   pressdeliver = true ;
 
    //   if(Math.abs(pos + 700000) > 50000) {
    //   pressdeliver = false;
    //   }
    // }  
     
    // if(!pressdeliver) {
    //   _talon.setSelectedSensorPosition(0);
    //   _talon.set(ControlMode.PercentOutput, 0);
    //   }
/////////////get in ///////////////////
  if(con.getXButtonPressed()){
      if(pressX==false){
        victorgetin.set(-0.29);
        pressX=true;
      }
      else if(pressX==true){
        victorgetin.set(0);
        pressX=false;
      }  
  }
//////////////get out///////////////////
  if(con.getAButtonPressed()){
    if(pressA==false){
      victorgetin.set(0.35);
      pressA=true;
    }
    else if(pressA==true){
      victorgetin.set(0);
      pressA=false;
    }  
  }
//////////////wheel///////////
  if(Math.abs(con.getY(Hand.kLeft))>0.1){
    victorDriveLeft.set(con.getY(Hand.kLeft)*-speed[i]);
  }
  else{
    victorDriveLeft.set(0);
  }
  if(Math.abs(con.getY(Hand.kRight))>0.1){
    victorDriveRight.set(con.getY(Hand.kRight)*speed[i]);
  }
  else{
    victorDriveRight.set(0);
  }
 
  if(con.getBumperPressed(Hand.kRight)){
    if(i<3){
      i++;
    }
  }
  if(con.getBumperPressed(Hand.kLeft)){
    if(i>0){
      i--;
    }
  }
}
 
  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }
 
  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }
 
  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }
 
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
    // System.out.println(pos);
    // _talon.set(ControlMode.PercentOutput, 0.3);
 
    
 
  }
}
 
