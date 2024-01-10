package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Kinematics {

    public final static double pracHALFSIDE=12.375*.0254;

    public final static double pracFLDX = 1*pracHALFSIDE;
    public final static double pracFLDY = 1*pracHALFSIDE;

    public final static double pracFRDX = 1*pracHALFSIDE;
    public final static double pracFRDY = -1*pracHALFSIDE;

    public final static double pracBLDX= -1*pracHALFSIDE;
    public final static double pracBLDY = 1*pracHALFSIDE;

    public final static double pracBRDX= -1*pracHALFSIDE;
    public final static double pracBRDY = -1*pracHALFSIDE;

    
    //The positions of the corners.  Must be in meters about the center of the robot
    //https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html

    public final static Translation2d locFL = new Translation2d(pracFLDX,pracFLDY);
    public final static Translation2d locFR = new Translation2d(pracFRDX,pracFRDY);
    public final static Translation2d locBL = new Translation2d(pracBLDX,pracBLDY);
    public final static Translation2d locBR = new Translation2d(pracBRDX,pracBRDY);
    
    // Creating my kinematics object using the module locations
    public static SwerveDriveKinematics kinPractice = new SwerveDriveKinematics(locFL, locFR, locBL, locBR);

    public static SwerveDriveKinematics getCentralPivotKinematics() {
        return kinPractice;
    }

    public static SwerveModuleState[] getToModuleStates(ChassisSpeeds desiredMotion){
        return kinPractice.toSwerveModuleStates(desiredMotion);
    }
}
