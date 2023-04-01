package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign {
    
    private static double input;
    private static double idk;
    private static double maxOutput = 0.5;
    private static double kP = -30; //TUNE THIS -Owen N | 30 works
    public static double output;


    //Basic P controller
    public static void align() {
        input = SwerveSubsystem.getYaw().getDegrees();

        idk = input/kP;

        if (idk > maxOutput) {
            output = maxOutput;
        } else if (idk < -maxOutput) {
            output = -maxOutput;
        } else {
            output = idk;
        }
    }

    public static void disable() {
        output = 0;
    }
}
