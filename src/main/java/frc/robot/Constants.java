package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

public final class Constants {
    public static class SwerveConstants {
        public static final double MAX_SPEED = 3.0;
        public static final PIDConstants autoDrivePID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants autoRotationPID = new PIDConstants(5.0, 0.0, 0.0);
    }
}