package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.math.Conversions;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmSubsystem;

public class AutoModes {
    private static ArmSubsystem m_arm = new ArmSubsystem();

    private static final double meterConversion = 39.37;

    public SequentialCommandGroup chosenAuto(int id) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

        TrajectoryConfig trajectoryConfigHalfSpeed = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond/2,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        switch (id) {

            case 1: //Just score Tip in L3
                xController.close();
                yController.close();

                return new SequentialCommandGroup(
                  
                new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.6)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> Timer.delay(.8)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> m_arm.setSol(false)),
                new InstantCommand(() -> Timer.delay(.6)),
                new InstantCommand(() -> m_arm.releaseGamePiece()),
                new InstantCommand(() -> Timer.delay(1)),
                new InstantCommand(() -> m_arm.endEffectorOpenLoop(.5)),
                new InstantCommand(() -> Timer.delay(.25)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.5)),
                new InstantCommand(() -> m_arm.goHome())//,
//                new InstantCommand(() -> Timer.delay(.25)),
//                new InstantCommand(() -> m_arm.chargeStationArmDown()),
//                new InstantCommand(() -> m_arm.setSol(true)),
//                new InstantCommand(() -> Timer.delay(.25)),
//                new InstantCommand(() -> m_arm.chargeStationArmDown())
                  );

            case 2: //Do nothing
                xController.close();
                yController.close();
                return null; //we seriously need to tune this one, its our most important auto

            case 3: //Just drive back 
                Trajectory driveBack = TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                    List.of(
                        new Translation2d(-1.0, 0.0)),
                    new Pose2d(-2.0, 0.0, Rotation2d.fromDegrees(0.0)),
                    trajectoryConfig);

                SwerveControllerCommand driveBackCommand = new SwerveControllerCommand(
                    driveBack,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                return new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.s_Swerve.resetOdometry(driveBack.getInitialPose())),
                    driveBackCommand
                );

            case 4: //Score balance

                Trajectory shootDriveTrajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(-0.1, 0.0, new Rotation2d(0.0)), 
                    List.of(
                        new Translation2d(-.5, -0.35)), //ADDED -OWEN N 3/17 @ 23:48
                    new Pose2d(-2.38, -0.35, Rotation2d.fromDegrees(22.0)), //was -2.38,-.35 -OWEN N 3/17 @ 23:49
                    trajectoryConfigHalfSpeed);
            
                SwerveControllerCommand scoreDriveCommand = new SwerveControllerCommand(
                    shootDriveTrajectory,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
                    
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
                    new InstantCommand(() -> m_arm.hold()),
                    new InstantCommand(() -> m_arm.prescore()),
                    new InstantCommand(() -> Timer.delay(.6)),
                    new InstantCommand(() -> m_arm.levelThree()),
                    new InstantCommand(() -> Timer.delay(.8)),
                    new InstantCommand(() -> m_arm.levelThree()),
                    new InstantCommand(() -> m_arm.setSol(false)),
                    new InstantCommand(() -> Timer.delay(.6)),
                    new InstantCommand(() -> m_arm.releaseGamePiece()),
                    new InstantCommand(() -> Timer.delay(1)),
                    new InstantCommand(() -> m_arm.endEffectorOpenLoop(.5)),
                    new InstantCommand(() -> Timer.delay(.25)),
                    new InstantCommand(() -> m_arm.prescore()),
                    new InstantCommand(() -> Timer.delay(.25)),
                    new InstantCommand(() -> m_arm.goHome()),
                    new InstantCommand(() -> Timer.delay(.25)),
                    new InstantCommand(() -> m_arm.traverse()),
                    new InstantCommand(() -> Timer.delay(.5)),
                    new InstantCommand(() -> m_arm.chargeStationArmDown()),
                    new InstantCommand(() -> m_arm.setSol(true)),
                    new InstantCommand(() -> Timer.delay(.25)),
                    new InstantCommand(() -> m_arm.chargeStationArmDown()),
//                    new InstantCommand(() -> RobotContainer.s_Swerve.autoBalance()), //BVN - I commented this out at 1:32pm 3/16 after practice match 5
                    scoreDriveCommand,
                    new InstantCommand(() -> RobotContainer.s_Swerve.autoBalance())
                    );

            case 5: //Right side 2 piece

            Trajectory driveToGP2 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-0.5, 0.0, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                List.of(
                    new Translation2d(-50.0/meterConversion, 0.0),
                    new Translation2d(-125.0/meterConversion, -12.0/meterConversion),
                    new Translation2d(-150.0/meterConversion, -18.0/meterConversion)),
                new Pose2d(-193.0/meterConversion, -18.0/meterConversion, Rotation2d.fromDegrees(0.0)), //AZER was -185, came up short sometimes
                trajectoryConfigHalfSpeed
            );

            Trajectory returnHome = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-185.0/meterConversion, -18.0/meterConversion, new Rotation2d(0.0)),
                List.of(
                    //new Translation2d(-170.0/meterConversion, -3.0/meterConversion),
                    new Translation2d(-50.0/meterConversion, -12.0/meterConversion), //azer was -30,-12
                    //                    new Translation2d(-20.0/meterConversion, -3.0/meterConversion),

                    new Translation2d(-28.0/meterConversion, -43.0/meterConversion)), 
                new Pose2d(0.1, -45.0/meterConversion, Rotation2d.fromDegrees(0.0)),
                trajectoryConfigHalfSpeed
            );

            Trajectory driveToGP3 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-0.5, -45.0/meterConversion, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                List.of(
                    new Translation2d(-30.0/meterConversion, 0.0),
                    new Translation2d(-125.0/meterConversion, -6.0/meterConversion),
                    new Translation2d(-180.0/meterConversion, -12.0/meterConversion)),
                new Pose2d(-190.0/meterConversion, -60.0/meterConversion, Rotation2d.fromDegrees(45)), //AZER was -185, came up short sometimes
                trajectoryConfig
            );
            SwerveControllerCommand driveToGP2Command = new SwerveControllerCommand(
                driveToGP2,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);

            SwerveControllerCommand returnCommand = new SwerveControllerCommand(
                returnHome,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);

                SwerveControllerCommand driveToGP3Command = new SwerveControllerCommand(
                    driveToGP3,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
                return new SequentialCommandGroup(
                    
//INITIAL SCORE SEQUENCE
                new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.3)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> Timer.delay(.6)),
                new InstantCommand(() -> m_arm.levelThree()),
//                new InstantCommand(() -> m_arm.setSol(false)),
                new InstantCommand(() -> Timer.delay(.6)),
                new InstantCommand(() -> m_arm.releaseGamePiece()),
                new InstantCommand(() -> Timer.delay(0.1)),
                new InstantCommand(() -> m_arm.endEffectorOpenLoop(-.6)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.intakeConePointIn()),
                new InstantCommand(() -> Timer.delay(.1)),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                new InstantCommand(() -> m_arm.endEffectorOpenLoop(.95)), //yeehaw
                
                driveToGP2Command,
//                new InstantCommand(() -> Timer.delay(2)),

//PRE-RETURN SEQUENCE        
                new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.goHome()),
                new InstantCommand(() -> m_arm.goHome()),
                new InstantCommand(() -> m_arm.setSol(true)),
                new InstantCommand(() -> Timer.delay(.7)),
                new InstantCommand(() -> m_arm.goHome()),
                new InstantCommand(() -> m_arm.goHome()),
                new InstantCommand(() -> m_arm.goHome()),
//                new InstantCommand(() -> Timer.delay(.2)),
                new InstantCommand(() -> m_arm.stopHoming()),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> m_arm.prescore()),
                
                returnCommand,

//FINAL SCORE SEQUENCE
                new InstantCommand(() -> m_arm.prescore()),
//                new InstantCommand(() -> Timer.delay(0.3)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> Timer.delay(0.3)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> m_arm.setSol(false)),
                new InstantCommand(() -> Timer.delay(1.1)),
                new InstantCommand(() -> m_arm.releaseGamePiece()),
                new InstantCommand(() -> m_arm.endEffectorOpenLoop(-0.1)),
                new InstantCommand(() -> Timer.delay(.25)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.goHome())/*,

                driveToGP3Command */

            );

            case 6: //1.5 piece
            
            Trajectory driveToGP26 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-0.5, 0.0, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                List.of(
                    new Translation2d(-50.0/meterConversion, 0.0),
                    new Translation2d(-125.0/meterConversion, 0.0),
                    new Translation2d(-150.0/meterConversion, -12.0/meterConversion)),
                new Pose2d(-190.0/meterConversion, -12.0/meterConversion, Rotation2d.fromDegrees(0.0)), 
                trajectoryConfig
            );

            Trajectory balance6 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-190.0/meterConversion, -12.0/meterConversion, new Rotation2d(0.0)),
                List.of(
                    new Translation2d(-155.0/meterConversion, -5.0/meterConversion)),
                new Pose2d(-145.0/meterConversion, -60.0/meterConversion, Rotation2d.fromDegrees(0.0)),
                trajectoryConfig
            );
            Trajectory balance16 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-185.0/meterConversion, -60.0/meterConversion, new Rotation2d(0.0)),
                List.of(
                    new Translation2d(-136.75/meterConversion, -60.0/meterConversion)), 
                new Pose2d(-82.6772/meterConversion, -60.0/meterConversion, Rotation2d.fromDegrees(0.0)),
                trajectoryConfig
            );

            SwerveControllerCommand driveToGP2Command6 = new SwerveControllerCommand(
                driveToGP26,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);

            SwerveControllerCommand balance6command = new SwerveControllerCommand(
                balance6,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);

            SwerveControllerCommand balance16command = new SwerveControllerCommand(
                balance16,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);

                return new SequentialCommandGroup(
                    
                new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.3)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> Timer.delay(.6)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> Timer.delay(.6)),
                new InstantCommand(() -> m_arm.releaseGamePiece()),
                new InstantCommand(() -> Timer.delay(0.1)),
                new InstantCommand(() -> m_arm.endEffectorOpenLoop(-.6)),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.intakeConePointIn()),
                new InstantCommand(() -> Timer.delay(.1)),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                
                driveToGP2Command6,

                new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.traverse()),
                new InstantCommand(() -> Timer.delay(.1)),
                new InstantCommand(() -> m_arm.goHome()),
                new InstantCommand(() -> Timer.delay(.2)),

                new InstantCommand(() -> m_arm.stopHoming()),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> m_arm.prescore()),

                balance6command,
                new InstantCommand(() -> Timer.delay(1)),
                balance16command,
                new InstantCommand(() -> Timer.delay(1))//,
//                new InstantCommand(() -> RobotContainer.s_Swerve.autoBalance())
            );        

            case 7: //Left side 2 piece

            Trajectory driveToGP27 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-0.5, 0.0, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                List.of(
                    new Translation2d(-50.0/meterConversion, 0.0),
                    new Translation2d(-125.0/meterConversion, 9.0/meterConversion), //AZER was -125,0
                    new Translation2d(-150.0/meterConversion, 18.0/meterConversion)),
                new Pose2d(-193.0/meterConversion, 18.0/meterConversion, Rotation2d.fromDegrees(0.0)), 
                trajectoryConfigHalfSpeed
            );

            Trajectory returnHome7 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-185.0/meterConversion, 18.0/meterConversion, new Rotation2d(0.0)),
                List.of(
                    //new Translation2d(-170.0/meterConversion, -3.0/meterConversion),
                    new Translation2d(-50.0/meterConversion, 15.0/meterConversion), //AZER was -30, 15
                    new Translation2d(-28.0/meterConversion, 43.0/meterConversion)), 
                new Pose2d(0.05, 45.0/meterConversion, Rotation2d.fromDegrees(0.0)),//
                trajectoryConfigHalfSpeed
            );
            
            Trajectory driveToGP37 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-0.1, 45.0/meterConversion, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                List.of(
                    new Translation2d(-20.0/meterConversion, 0.0),
                    new Translation2d(-125.0/meterConversion, 9.0/meterConversion), //AZER was -125,0
                    new Translation2d(-180.0/meterConversion, 18.0/meterConversion)),
                new Pose2d(-190.0/meterConversion, 60.0/meterConversion, Rotation2d.fromDegrees(-45)), 
                trajectoryConfig
            );

            SwerveControllerCommand driveToGP27Command = new SwerveControllerCommand(
                driveToGP27,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);

            SwerveControllerCommand return7Command = new SwerveControllerCommand(
                returnHome7,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);
        
            SwerveControllerCommand driveToGP37Command = new SwerveControllerCommand(
                driveToGP37,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);

                return new SequentialCommandGroup(
                    
//INITIAL SCORE SEQUENCE
                    
                new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.3)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> Timer.delay(.6)),
                new InstantCommand(() -> m_arm.levelThree()),
//                new InstantCommand(() -> m_arm.setSol(false)),
                new InstantCommand(() -> Timer.delay(.6)),
                new InstantCommand(() -> m_arm.releaseGamePiece()),
                new InstantCommand(() -> Timer.delay(0.1)),
                new InstantCommand(() -> m_arm.endEffectorOpenLoop(-.6)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.intakeConePointIn()),
                new InstantCommand(() -> Timer.delay(.1)),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),

//                new InstantCommand(() -> m_arm.traverseAuto()),
//                new InstantCommand(() -> m_arm.traverseAuto()),
//                new InstantCommand(() -> m_arm.traverseAuto()),
                new InstantCommand(() -> m_arm.endEffectorOpenLoop(.85)), //yeehaw was 0.95
                
                //new InstantCommand(() -> Timer.delay(3.0)), //for test w/o path
               
                
//DRIVE TO GAMEPIECE                
/*danger-uncomment*/                driveToGP27Command,

//PRE-RETURN SEQUENCE        
                new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.goHome()),
                new InstantCommand(() -> m_arm.goHome()),
                new InstantCommand(() -> m_arm.setSol(true)),
                new InstantCommand(() -> Timer.delay(.7)),
                new InstantCommand(() -> m_arm.goHome()),
                new InstantCommand(() -> m_arm.goHome()),
                new InstantCommand(() -> m_arm.goHome()),
//                new InstantCommand(() -> Timer.delay(.2)),
                new InstantCommand(() -> m_arm.stopHoming()),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> m_arm.prescore()),
                
                return7Command,

//FINAL SCORE SEQUENCE
                new InstantCommand(() -> m_arm.prescore()),
//                new InstantCommand(() -> Timer.delay(0.3)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> Timer.delay(0.3)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> m_arm.setSol(false)),
                new InstantCommand(() -> Timer.delay(1.1)),
                new InstantCommand(() -> m_arm.releaseGamePiece()),
                new InstantCommand(() -> m_arm.endEffectorOpenLoop(-0.1)),
                new InstantCommand(() -> Timer.delay(.25)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.goHome())/*,
                driveToGP37Command*/
                
            );

            case 8: //Tuning
                Trajectory tuningTraj = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(),
                    List.of(
                        new Translation2d(-0.5, -0.5)),
                    new Pose2d(-1, -1, Rotation2d.fromDegrees(90.0)),
                    trajectoryConfig);

                SwerveControllerCommand tuningSwerveControllerCommand = new SwerveControllerCommand(
                    tuningTraj,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                return new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.s_Swerve.resetOdometry(tuningTraj.getInitialPose())),
                    tuningSwerveControllerCommand
                );

                case 9: //1.5 piece
            
            Trajectory driveToGP29 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-0.5, 0.0, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                List.of(
                    new Translation2d(-50.0/meterConversion, 0.0),
                    new Translation2d(-125.0/meterConversion, 0.0),
                    new Translation2d(-150.0/meterConversion, 12.0/meterConversion)),
                new Pose2d(-185.0/meterConversion, 12.0/meterConversion, Rotation2d.fromDegrees(0.0)), 
                trajectoryConfig
            );

            Trajectory balance9 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-190.0/meterConversion, 12.0/meterConversion, new Rotation2d(0.0)),
                List.of(
                    new Translation2d(-155.0/meterConversion, 5.0/meterConversion)),
                new Pose2d(-145.0/meterConversion, 60.0/meterConversion, Rotation2d.fromDegrees(0.0)),
                trajectoryConfig
            );
            Trajectory balance19 = TrajectoryGenerator.generateTrajectory( 
                new Pose2d(-190.0/meterConversion, 60.0/meterConversion, new Rotation2d(0.0)),
                List.of(
                    new Translation2d(-136.75/meterConversion, -60.0/meterConversion)), 
                new Pose2d(-82.6772/meterConversion, 60.0/meterConversion, Rotation2d.fromDegrees(0.0)),
                trajectoryConfig
            );

            SwerveControllerCommand driveToGP2Command9 = new SwerveControllerCommand(
                driveToGP29,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);

            SwerveControllerCommand balance9command = new SwerveControllerCommand(
                balance9,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);

            SwerveControllerCommand balance19command = new SwerveControllerCommand(
                balance19,
                RobotContainer.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                RobotContainer.s_Swerve::setModuleStates,
                RobotContainer.s_Swerve);

                return new SequentialCommandGroup(
                    
                new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.3)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> Timer.delay(.6)),
                new InstantCommand(() -> m_arm.levelThree()),
                new InstantCommand(() -> Timer.delay(.6)),
                new InstantCommand(() -> m_arm.releaseGamePiece()),
                new InstantCommand(() -> Timer.delay(0.1)),
                new InstantCommand(() -> m_arm.endEffectorOpenLoop(-.6)),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.intakeConePointIn()),
                new InstantCommand(() -> Timer.delay(.1)),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                new InstantCommand(() -> m_arm.intakeConePointInAuto()),
                
                driveToGP2Command9,

                new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
                new InstantCommand(() -> m_arm.hold()),
                new InstantCommand(() -> m_arm.traverse()),
                new InstantCommand(() -> Timer.delay(.1)),
                new InstantCommand(() -> m_arm.goHome()),
                new InstantCommand(() -> Timer.delay(.2)),

                new InstantCommand(() -> m_arm.stopHoming()),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> Timer.delay(.125)),
                new InstantCommand(() -> m_arm.prescore()),
                new InstantCommand(() -> m_arm.prescore()),

                balance9command,
                new InstantCommand(() -> Timer.delay(1)),
                balance19command,
                new InstantCommand(() -> Timer.delay(1))//,
//                new InstantCommand(() -> RobotContainer.s_Swerve.autoBalance())
            ); 

            case 10: //2 piece cube left

            Trajectory driveToGP210 = TrajectoryGenerator.generateTrajectory( 
                    new Pose2d(-0.5, 0.0, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                    List.of(
                        new Translation2d(-50.0/meterConversion, 0.0),
                        new Translation2d(-125.0/meterConversion, 6.0/meterConversion),
                    new Translation2d(-150.0/meterConversion, 15.0/meterConversion)),
                    new Pose2d(-185.0/meterConversion, 15.0/meterConversion, Rotation2d.fromDegrees(0.0)), 
                    trajectoryConfig
                );
    
                Trajectory returnHome10 = TrajectoryGenerator.generateTrajectory( 
                    new Pose2d(-185.0/meterConversion, 15.0/meterConversion, new Rotation2d(0.0)),
                    List.of(

                    new Translation2d(-50.0/meterConversion, 15.0/meterConversion), 
                    new Translation2d(-30.0/meterConversion, 18.0/meterConversion)), 
                        //new Translation2d(-30.0/meterConversion, 12.0/meterConversion), 
                        //new Translation2d(-30.0/meterConversion, 6.0/meterConversion)), 
                    new Pose2d(0.1, 20.0/meterConversion, Rotation2d.fromDegrees(0.0)),
                    trajectoryConfig
                );
    
                Trajectory balance10 = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(-0.05/meterConversion, 20.0/meterConversion, new Rotation2d(0.0)), 
                        List.of(
                            new Translation2d(-.5, 65.0/meterConversion)), //ADDED -OWEN N 3/17 @ 23:48
                        new Pose2d(-2.40, 65.0/meterConversion, Rotation2d.fromDegrees(22.0)), //was -2.38,-.35 -OWEN N 3/17 @ 23:49
                        trajectoryConfigHalfSpeed);
                
                    SwerveControllerCommand balance10Command = new SwerveControllerCommand(
                        balance10,
                        RobotContainer.s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        xController,
                        yController,
                        thetaController,
                        RobotContainer.s_Swerve::setModuleStates,
                        RobotContainer.s_Swerve);
    
                SwerveControllerCommand driveToGP210Command = new SwerveControllerCommand(
                    driveToGP210,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
    
                SwerveControllerCommand return10Command = new SwerveControllerCommand(
                    returnHome10,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
    
                    return new SequentialCommandGroup(
                        
                    scoreCone(),
    
                    new InstantCommand(() -> m_arm.intakeCubeAuto()),
                    
                    //new InstantCommand(() -> Timer.delay(3.0)), //for test w/o path
                   
                    
    //DRIVE TO GAMEPIECE                
    /*danger-uncomment*/  driveToGP210Command,
    
    //PRE-RETURN SEQUENCE                
                    new InstantCommand(() -> m_arm.intakeCubeAuto()),
                    new InstantCommand(() -> m_arm.setAutoGamePiece(1)),
                    
                    new InstantCommand(() -> Timer.delay(0.5)),
                    new InstantCommand(() -> m_arm.hold()),
                    new InstantCommand(() -> m_arm.prescoreAuto()),
    
    /*danger-uncomment*/  return10Command,
    //new InstantCommand(() -> Timer.delay(3.0)), //for test w/o path
    
    //FINAL SCORE SEQUENCE
                    new InstantCommand(() -> m_arm.l3Auto()),
                    new InstantCommand(() -> Timer.delay(0.5)),
                    new InstantCommand(() -> m_arm.endEffectorOpenLoop(0.7)),
                    new InstantCommand(() -> Timer.delay(0.5)),
                    new InstantCommand(() -> m_arm.endEffectorOpenLoop(0.0)),
                    //new InstantCommand(() -> m_arm.intakeConePointIn())
                    new InstantCommand(() -> m_arm.chargeStationArmDown()),
    
    /*danger-uncomment*/                balance10Command,
                    new InstantCommand(() -> RobotContainer.s_Swerve.autoBalance())
    
                );

                case 11: //2 piece cube right

                Trajectory driveToGP211 = TrajectoryGenerator.generateTrajectory( 
                        new Pose2d(-0.5, 0.0, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                        List.of(
                            new Translation2d(-50.0/meterConversion, 0.0),
                            new Translation2d(-125.0/meterConversion, -6.0/meterConversion),
                            new Translation2d(-150.0/meterConversion, -16.0/meterConversion)),
                        new Pose2d(-185.0/meterConversion, -16.0/meterConversion, Rotation2d.fromDegrees(0.0)), 
                        trajectoryConfig
                    );
        
                    Trajectory returnHome11 = TrajectoryGenerator.generateTrajectory( 
                        new Pose2d(-185.0/meterConversion, -16.0/meterConversion, new Rotation2d(0.0)),
                        List.of(
                            new Translation2d(-50.0/meterConversion, -15.0/meterConversion), 
                            new Translation2d(-30.0/meterConversion, -18.0/meterConversion)), 
                            //new Translation2d(-30.0/meterConversion, -12.0/meterConversion),
                            //new Translation2d(-30.0/meterConversion, -6.0/meterConversion)), 
                        new Pose2d(0.1, -20.0/meterConversion, Rotation2d.fromDegrees(0.0)),
                        trajectoryConfig
                    );
        
                    Trajectory balance11 = TrajectoryGenerator.generateTrajectory(
                            new Pose2d(-0.05/meterConversion, -20.0/meterConversion, new Rotation2d(0.0)), 
                            List.of(
                                new Translation2d(-.5, -65.0/meterConversion)), //ADDED -OWEN N 3/17 @ 23:48
                            new Pose2d(-2.4, -65.0/meterConversion, Rotation2d.fromDegrees(22.0)), //was -2.38,-.35 -OWEN N 3/17 @ 23:49
                            trajectoryConfigHalfSpeed);
                    
                        SwerveControllerCommand balance11Command = new SwerveControllerCommand(
                            balance11,
                            RobotContainer.s_Swerve::getPose,
                            Constants.Swerve.swerveKinematics,
                            xController,
                            yController,
                            thetaController,
                            RobotContainer.s_Swerve::setModuleStates,
                            RobotContainer.s_Swerve);
        
                    SwerveControllerCommand driveToGP211Command = new SwerveControllerCommand(
                        driveToGP211,
                        RobotContainer.s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        xController,
                        yController,
                        thetaController,
                        RobotContainer.s_Swerve::setModuleStates,
                        RobotContainer.s_Swerve);
        
                    SwerveControllerCommand return11Command = new SwerveControllerCommand(
                        returnHome11,
                        RobotContainer.s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        xController,
                        yController,
                        thetaController,
                        RobotContainer.s_Swerve::setModuleStates,
                        RobotContainer.s_Swerve);
        
                        return new SequentialCommandGroup(
                            
                        scoreCone(),
        
                        new InstantCommand(() -> m_arm.intakeCubeAuto()),
                        
                        //new InstantCommand(() -> Timer.delay(3.0)), //for test w/o path
                       
                        
        //DRIVE TO GAMEPIECE                
        /*danger-uncomment*/  driveToGP211Command,
        
        //PRE-RETURN SEQUENCE                
                        new InstantCommand(() -> m_arm.intakeCubeAuto()),
                        new InstantCommand(() -> m_arm.setAutoGamePiece(1)),
                        
                        new InstantCommand(() -> Timer.delay(0.5)),
                        new InstantCommand(() -> m_arm.hold()),
                        new InstantCommand(() -> m_arm.prescoreAuto()),
        
        /*danger-uncomment*/  return11Command,
        //new InstantCommand(() -> Timer.delay(3.0)), //for test w/o path
        
        //FINAL SCORE SEQUENCE
                        new InstantCommand(() -> m_arm.l3Auto()),
                        new InstantCommand(() -> Timer.delay(0.5)),
                        new InstantCommand(() -> m_arm.endEffectorOpenLoop(0.7)),
                        new InstantCommand(() -> Timer.delay(0.5)),
                        new InstantCommand(() -> m_arm.endEffectorOpenLoop(0.0)),
                        //new InstantCommand(() -> m_arm.intakeConePointIn())
                        new InstantCommand(() -> m_arm.chargeStationArmDown()),  
                        //new InstantCommand(() -> m_arm.chargeStationArmDown())//,
        
        /*danger-uncomment*/                balance11Command,
                        new InstantCommand(() -> RobotContainer.s_Swerve.autoBalance())
                    );

                    case 12: // Right 2.5 Piece

                    Trajectory driveToGP212 = TrajectoryGenerator.generateTrajectory( 
                            new Pose2d(-0.5, 0.0, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                            List.of(
                                new Translation2d(-50.0/meterConversion, 0.0),
                                new Translation2d(-125.0/meterConversion, -6.0/meterConversion),
                                new Translation2d(-150.0/meterConversion, -16.0/meterConversion)),
                            new Pose2d(-185.0/meterConversion, -16.0/meterConversion, Rotation2d.fromDegrees(0.0)), 
                            trajectoryConfig
                        );
            
                        Trajectory returnHome12 = TrajectoryGenerator.generateTrajectory( 
                            new Pose2d(-185.0/meterConversion, -16.0/meterConversion, new Rotation2d(0.0)),
                            List.of(
                                new Translation2d(-50.0/meterConversion, -15.0/meterConversion), 
                                new Translation2d(-30.0/meterConversion, -18.0/meterConversion)), 
                                //new Translation2d(-30.0/meterConversion, -12.0/meterConversion),
                                //new Translation2d(-30.0/meterConversion, -6.0/meterConversion)), 
                            new Pose2d(0.1, -20.0/meterConversion, Rotation2d.fromDegrees(0.0)),
                            trajectoryConfig
                        );
            
                        Trajectory driveToGP312 = TrajectoryGenerator.generateTrajectory(
                            new Pose2d(-0.5, -20.0/meterConversion, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                            List.of(
                                new Translation2d(-30.0/meterConversion, 0.0),
                                new Translation2d(-125.0/meterConversion, -6.0/meterConversion),
                                new Translation2d(-180.0/meterConversion, -12.0/meterConversion)),
                            new Pose2d(-190.0/meterConversion, -60.0/meterConversion, Rotation2d.fromDegrees(45)), //AZER was -185, came up short sometimes
                            trajectoryConfig);
                        
                            SwerveControllerCommand driveToGP312Command = new SwerveControllerCommand(
                                driveToGP312,
                                RobotContainer.s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                xController,
                                yController,
                                thetaController,
                                RobotContainer.s_Swerve::setModuleStates,
                                RobotContainer.s_Swerve);
            
                        SwerveControllerCommand driveToGP212Command = new SwerveControllerCommand(
                            driveToGP212,
                            RobotContainer.s_Swerve::getPose,
                            Constants.Swerve.swerveKinematics,
                            xController,
                            yController,
                            thetaController,
                            RobotContainer.s_Swerve::setModuleStates,
                            RobotContainer.s_Swerve);
            
                        SwerveControllerCommand return12Command = new SwerveControllerCommand(
                            returnHome12,
                            RobotContainer.s_Swerve::getPose,
                            Constants.Swerve.swerveKinematics,
                            xController,
                            yController,
                            thetaController,
                            RobotContainer.s_Swerve::setModuleStates,
                            RobotContainer.s_Swerve);
            
                            return new SequentialCommandGroup(
                                
                            scoreCone(),
            
                            new InstantCommand(() -> m_arm.intakeCubeAuto()),
                            
                            //new InstantCommand(() -> Timer.delay(3.0)), //for test w/o path
                           
                            
            //DRIVE TO GAMEPIECE                
            /*danger-uncomment*/  driveToGP212Command,
            
            //PRE-RETURN SEQUENCE                
                            new InstantCommand(() -> m_arm.intakeCubeAuto()),
                            new InstantCommand(() -> m_arm.setAutoGamePiece(1)),
                            
                            new InstantCommand(() -> Timer.delay(0.5)),
                            new InstantCommand(() -> m_arm.hold()),
                            new InstantCommand(() -> m_arm.prescoreAuto()),
            
            /*danger-uncomment*/  return12Command,
            //new InstantCommand(() -> Timer.delay(3.0)), //for test w/o path
            
            //FINAL SCORE SEQUENCE
                            new InstantCommand(() -> m_arm.l3Auto()),
                            new InstantCommand(() -> Timer.delay(0.5)),
                            new InstantCommand(() -> m_arm.endEffectorOpenLoop(0.7)),
                            new InstantCommand(() -> Timer.delay(0.5)),
                            new InstantCommand(() -> m_arm.endEffectorOpenLoop(0.0)),
                            new InstantCommand(() -> m_arm.intakeConePointIn()),
            
            /*danger-uncomment*/               driveToGP312Command
                        );

                        case 13: // Left 2.5 Piece

                        Trajectory driveToGP213 = TrajectoryGenerator.generateTrajectory( 
                                new Pose2d(-0.5, 0.0, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                                List.of(
                                    new Translation2d(-50.0/meterConversion, 0.0),
                                    new Translation2d(-125.0/meterConversion, 6.0/meterConversion),
                                    new Translation2d(-150.0/meterConversion, 16.0/meterConversion)),
                                new Pose2d(-185.0/meterConversion, 16.0/meterConversion, Rotation2d.fromDegrees(0.0)), 
                                trajectoryConfig
                            );
                
                            Trajectory returnHome13 = TrajectoryGenerator.generateTrajectory( 
                                new Pose2d(-185.0/meterConversion, -16.0/meterConversion, new Rotation2d(0.0)),
                                List.of(
                                    new Translation2d(-50.0/meterConversion, 15.0/meterConversion), 
                                    new Translation2d(-30.0/meterConversion, 18.0/meterConversion)), 
                                    //new Translation2d(-30.0/meterConversion, -12.0/meterConversion),
                                    //new Translation2d(-30.0/meterConversion, -6.0/meterConversion)), 
                                new Pose2d(0.1, 20.0/meterConversion, Rotation2d.fromDegrees(0.0)),
                                trajectoryConfig
                            );
                
                            Trajectory driveToGP313 = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(-0.5, -20.0/meterConversion, new Rotation2d(0.0)), //start furhter back to avoid the weird swivel at the start
                                List.of(
                                    new Translation2d(-30.0/meterConversion, 0.0),
                                    new Translation2d(-125.0/meterConversion, 6.0/meterConversion),
                                    new Translation2d(-180.0/meterConversion, 12.0/meterConversion)),
                                new Pose2d(-190.0/meterConversion, 60.0/meterConversion, Rotation2d.fromDegrees(-45)), //AZER was -185, came up short sometimes
                                trajectoryConfig);
                            
                                SwerveControllerCommand driveToGP313Command = new SwerveControllerCommand(
                                    driveToGP313,
                                    RobotContainer.s_Swerve::getPose,
                                    Constants.Swerve.swerveKinematics,
                                    xController,
                                    yController,
                                    thetaController,
                                    RobotContainer.s_Swerve::setModuleStates,
                                    RobotContainer.s_Swerve);
                
                            SwerveControllerCommand driveToGP213Command = new SwerveControllerCommand(
                                driveToGP213,
                                RobotContainer.s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                xController,
                                yController,
                                thetaController,
                                RobotContainer.s_Swerve::setModuleStates,
                                RobotContainer.s_Swerve);
                
                            SwerveControllerCommand return13Command = new SwerveControllerCommand(
                                returnHome13,
                                RobotContainer.s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                xController,
                                yController,
                                thetaController,
                                RobotContainer.s_Swerve::setModuleStates,
                                RobotContainer.s_Swerve);
                
                                return new SequentialCommandGroup(
                                    
                                scoreCone(),
                
                                new InstantCommand(() -> m_arm.intakeCubeAuto()),
                                
                                //new InstantCommand(() -> Timer.delay(3.0)), //for test w/o path
                               
                                
                //DRIVE TO GAMEPIECE                
                /*danger-uncomment*/  driveToGP213Command,
                
                //PRE-RETURN SEQUENCE                
                                new InstantCommand(() -> m_arm.intakeCubeAuto()),
                                new InstantCommand(() -> m_arm.setAutoGamePiece(1)),
                                
                                new InstantCommand(() -> Timer.delay(0.5)),
                                new InstantCommand(() -> m_arm.hold()),
                                new InstantCommand(() -> m_arm.prescoreAuto()),
                
                /*danger-uncomment*/  return13Command,
                //new InstantCommand(() -> Timer.delay(3.0)), //for test w/o path
                
                //FINAL SCORE SEQUENCE
                                new InstantCommand(() -> m_arm.l3Auto()),
                                new InstantCommand(() -> Timer.delay(0.5)),
                                new InstantCommand(() -> m_arm.endEffectorOpenLoop(0.7)),
                                new InstantCommand(() -> Timer.delay(0.5)),
                                new InstantCommand(() -> m_arm.endEffectorOpenLoop(0.0)),
                                new InstantCommand(() -> m_arm.intakeConePointIn()),
                
                /*danger-uncomment*/               driveToGP313Command
                            );
        }

        xController.close();
        yController.close();
        return null;
    }

    private static SequentialCommandGroup scoreCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_arm.autoPrepare()),
            new InstantCommand(() -> m_arm.setAutoGamePiece(2)),
            new InstantCommand(() -> m_arm.hold()),
            new InstantCommand(() -> m_arm.prescore()),
            new InstantCommand(() -> Timer.delay(.3)),
            new InstantCommand(() -> m_arm.levelThree()),
            new InstantCommand(() -> Timer.delay(.6)),
            new InstantCommand(() -> m_arm.levelThree()),
            new InstantCommand(() -> Timer.delay(.6)),
            new InstantCommand(() -> m_arm.releaseGamePiece()),
            new InstantCommand(() -> Timer.delay(0.1)),
            new InstantCommand(() -> m_arm.endEffectorOpenLoop(-.6)),
            new InstantCommand(() -> m_arm.prescore()),
            new InstantCommand(() -> Timer.delay(.125))
        );
    }
}