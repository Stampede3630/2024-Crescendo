// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.AllianceDependent;
import frc.robot.util.Region2D;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class FieldConstants {
        public static final AllianceDependent<Translation3d> SPEAKER_POSITION = new AllianceDependent<>(new Translation3d(.2, 5.55, 2.11), new Translation3d(16.33, 5.55, 2.11)); // POSITION OF THE SPEAKER (BLUE SIDE)
        public static final AllianceDependent<Region2D> AMP_REGION = new AllianceDependent<>(new Region2D(0.5, 8.25, 3.69, 6.98), new Region2D(16.5, 8.25, 12.93, 6.98));
    }
}
