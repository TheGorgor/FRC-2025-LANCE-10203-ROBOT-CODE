// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */

public class Robot extends TimedRobot {
  
  
  
  Thread m_visionThread;

  private static final String kDefaultAuto = "Default";
  private static final String kDuzOtonom = "Düz Gelen Otonom";
  private static final String kSolKoseOtonom = "Sol Köşeden Gelen Otonom";
  private static final String kSagKoseOtonom = "Sağ Köşeden Gelen Otonom";
  
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  
  private final SparkMax leftLeader = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax leftFollower = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax rightLeader = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax rightFollower = new SparkMax(4, MotorType.kBrushed);

  private final SparkMax rollerMotor = new SparkMax(5, MotorType.kBrushed);
 
  private final DifferentialDrive myDrive = new DifferentialDrive(leftLeader, rightLeader);

  private final SparkMaxConfig driveConfig = new SparkMaxConfig();
  
  private final SparkMaxConfig rollerConfig = new SparkMaxConfig();
  //rollerMotor configurationu oluşturduk
  
  private final Timer timer1 = new Timer();

  private final XboxController gamepad0 = new XboxController(0);
  //private final XboxController gamepad1 = new XboxController(1);
  //yeni gamepad için
  
  private double driveSpeed = 1;
  

  private final double SETTING_ROLLER_EJECT_VALUE = 0.35;
  private double ROLLER_EJECT_VALUE = 0;
  private double rollerMotorSinir = .5;
  //rollerMotor.set()deki parametreyi doldurmak için
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
     m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Düz Gelen Otonom", kDuzOtonom);
    m_chooser.addOption("Sol Köşeden Gelen Otonom", kSolKoseOtonom);
    m_chooser.addOption("Sağ Köşeden Gelen Otonom", kSagKoseOtonom);
    
    
    SmartDashboard.putData("Auto choices", m_chooser);
  
    driveConfig.smartCurrentLimit(60);
    driveConfig.voltageCompensation(12);

    driveConfig.follow(leftLeader);
    leftFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.follow(rightLeader);
    rightFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    driveConfig.disableFollowerMode();
    rightLeader.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    //driveConfig.inverted(true); eğer robot geriye doğu gittiyse
    leftLeader.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    rollerConfig.smartCurrentLimit(60);
    rollerConfig.voltageCompensation(10);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //rollerMotor için

    timer1.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotInit(){
    System.out.println("Robot açılıyor");
    checkAlliance();
  
  }

  public void checkAlliance() {
    
    Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
    
    
    if (ally.isPresent()) {
        
      if (ally.get() == DriverStation.Alliance.Red) {
        System.out.println("İttifak Rengi: Red");
        
      }
      
      else if (ally.get() == DriverStation.Alliance.Blue) {
        System.out.println("İttifak Rengi: Blue");
        
      }
      
     
    } 
    
    else {
      System.out.println("İttifak Rengi: Not Decided");
      
    }
    
    
  }
  
  @Override
  public void robotPeriodic() {
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    System.out.println("Otonom Modu Seçildi");
    timer1.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      
      case kDefaultAuto:
      
      case kSolKoseOtonom:
        if(timer1.get() < 5.4){
          myDrive.tankDrive(.435,-.435);
        }
        else if(timer1.get() < 5.7){
          myDrive.tankDrive(0,0);
        }
        else if(timer1.get() < 6.2){
          rollerMotor.set(0.5);
        }  
        else if(timer1.get() < 7){
          rollerMotor.set(0);
          myDrive.tankDrive(-.5,.5);
        }
        else if(timer1.get() < 7.4){
          myDrive.tankDrive(.5,.5);
        }
        else if (timer1.get() < 11){
          myDrive.tankDrive(.6,-.6);
        }
        else {
          myDrive.tankDrive(0,0);
          rollerMotor.set(0);
        }
        break;
        
      case kSagKoseOtonom:
        if(timer1.get() < 3){
          myDrive.tankDrive(.5,-.45);
        }
        else if(timer1.get() < 4){
          myDrive.tankDrive(.35,.35);
        }
        else if(timer1.get() < 4.3){
          myDrive.tankDrive(.5,-.5);
        }
        
        break;
      case kDuzOtonom:
        if(timer1.get() < 2.45){
          myDrive.tankDrive(.4925,-.4925);
        }
        else if(timer1.get() < 2.9){
          myDrive.tankDrive(0, 0);
          
        }
        else if(timer1.get() < 3.25){
          rollerMotor.set(.5);
          
        }
        else if(timer1.get() < 4){
          rollerMotor.set(0);
          myDrive.tankDrive(-.45, .45);
        }
        else if(timer1.get() < 4.6){
          myDrive.tankDrive(-.5,-.5);
        }  
        else if(timer1.get() < 8.95){
          myDrive.tankDrive(.5, -.425);
        }
        else if(timer1.get() < 9){
          myDrive.tankDrive(0,0);
        }
        else{
          rollerMotor.set(0);
          myDrive.tankDrive(0,0);
        }
        break;
      default:
        //timer1.get() < x şeklinde
        //rollerMotor.set(x) rollerMotor çalışır
        //myDrive.tankDrive(.5, .5);
        //düz gitmesini sağlayacak
        //---------
        //myDrive.tankDrive(leftSpeed: .5, -.5);
        //dönmesini sağlayacak kendi etrafında
        //---------
        //leftLeader.set(.5);
        //rightLeader.set(.5); 
        //(test amaçlı) düz gitmiş olacak
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  
  
  
  @Override
  public void teleopInit() {
    System.out.println("Teleop Modu Başlatıldı");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    if (gamepad0.getLeftBumperButton()){
      //L1e basıldığında
      driveSpeed = 0.425;
      
    }
    if (gamepad0.getRightBumperButton()){
      //R1e basıldığında
      driveSpeed = 1;
      
    }
    
    
    myDrive.arcadeDrive(-gamepad0.getRightX()*driveSpeed, gamepad0.getLeftY()*driveSpeed);
    
    
    ROLLER_EJECT_VALUE = gamepad0.getRightTriggerAxis()*rollerMotorSinir - gamepad0.getLeftTriggerAxis()*rollerMotorSinir;
    rollerMotor.set(ROLLER_EJECT_VALUE);
  }
    
    
    
    //driveSpeed değişkenini tanımlamıştık onun değeri ile bir hız sınırı koyuyoruz çarparak
    //ki kontrol etmesi zorlaşmasın
    //myDrive.tankDrive(-gamepad0.getLeftY() , -gamepad0.getRightY());
    //geriye giderse diye '-' koyduk, sürmeye bu çok elverişli değil
  

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}
    
  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    System.out.println("Test Modu Başlatılıyor.");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
    if (timer1.get() < 4){
      myDrive.tankDrive(.5 , .5);
    }
    
    else if(timer1.get() < 5.5){
      rollerMotor.set(.7);
    }
    
    else if(timer1.get() < 7){
      myDrive.tankDrive(-.5, -.5);
    }
    
    else if(timer1.get() < 8.5){
      myDrive.tankDrive(.5,-.5);
    }
    
    else if(timer1.get() < 11){
      myDrive.tankDrive(-.3, 0.6);
    }
    
    else if(timer1.get() < 13){
      myDrive.tankDrive(-0.5,0.5);
    }
    
    else if(timer1.get() < 15){
      myDrive.tankDrive(0.5, 0.5);
    }
    
    else if(timer1.get() == 15){
      myDrive.tankDrive(0,0);
      rollerMotor.set(0);
    }
    
    else{
      if (gamepad0.getLeftBumperButton()){
        //L1e basıldığında
        driveSpeed = 0.425;
        
      }
      if (gamepad0.getRightBumperButton()){
        //R1e basıldığında
        driveSpeed = 1;
      
      myDrive.arcadeDrive(-gamepad0.getRightX()*driveSpeed, gamepad0.getLeftY()*driveSpeed);

      ROLLER_EJECT_VALUE = gamepad0.getRightTriggerAxis()*rollerMotorSinir - gamepad0.getLeftTriggerAxis()*rollerMotorSinir;
      rollerMotor.set(ROLLER_EJECT_VALUE);  
      }

    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    System.out.println("Simülasyon Modu Başlatılıyor.");
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
