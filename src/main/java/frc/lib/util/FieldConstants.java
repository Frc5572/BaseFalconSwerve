/*
 * MIT License
 *
 * Copyright (c) 2023 FRC 6328
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

// Modified by 5572 to include fieldSdf and associated Signed Distance Field functions

package frc.lib.util;

import java.util.Map;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Field Constants
 */
public class FieldConstants {

    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double tapeWidth = Units.inchesToMeters(2.0);

    /**
     * Dimensions for community and charging station, including the tape.
     */
    public static final class Community {
        // Region dimensions
        public static final double innerX = 0.0;
        // Tape to the left of charging station
        public static final double midX = Units.inchesToMeters(132.375);
        // Tape to the right of charging station
        public static final double outerX = Units.inchesToMeters(193.25);
        public static final double leftY = Units.feetToMeters(18.0);
        public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;
        public static final double rightY = 0.0;
        public static final Translation2d[] regionCorners = new Translation2d[] {
            new Translation2d(innerX, rightY), new Translation2d(innerX, leftY),
            new Translation2d(midX, leftY), new Translation2d(midX, midY),
            new Translation2d(outerX, midY), new Translation2d(outerX, rightY)};

        // Charging station dimensions
        public static final double chargingStationLength = Units.inchesToMeters(76.125);
        public static final double chargingStationWidth = Units.inchesToMeters(97.25);
        public static final double chargingStationOuterX = outerX - tapeWidth;
        public static final double chargingStationInnerX =
            chargingStationOuterX - chargingStationLength;
        public static final double chargingStationLeftY = midY - tapeWidth;
        public static final double chargingStationRightY =
            chargingStationLeftY - chargingStationWidth;
        public static final Translation2d[] chargingStationCorners =
            new Translation2d[] {new Translation2d(chargingStationInnerX, chargingStationRightY),
                new Translation2d(chargingStationInnerX, chargingStationLeftY),
                new Translation2d(chargingStationOuterX, chargingStationRightY),
                new Translation2d(chargingStationOuterX, chargingStationLeftY)};

        // Cable bump
        public static final double cableBumpInnerX =
            innerX + Grids.outerX + Units.inchesToMeters(95.25);
        public static final double cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
        public static final Translation2d[] cableBumpCorners =
            new Translation2d[] {new Translation2d(cableBumpInnerX, 0.0),
                new Translation2d(cableBumpInnerX, chargingStationRightY),
                new Translation2d(cableBumpOuterX, 0.0),
                new Translation2d(cableBumpOuterX, chargingStationRightY)};
    }

    /**
     * Dimensions for grids and nodes
     */
    public static final class Grids {
        // X layout
        public static final double outerX = Units.inchesToMeters(54.25);
        // Centered when under cube nodes
        public static final double lowX = outerX - (Units.inchesToMeters(14.25) / 2.0);
        public static final double midX = outerX - Units.inchesToMeters(22.75);
        public static final double highX = outerX - Units.inchesToMeters(39.75);

        // Y layout
        public static final int nodeRowCount = 9;
        public static final double nodeFirstY = Units.inchesToMeters(20.19);
        public static final double nodeSeparationY = Units.inchesToMeters(22.0);

        // Z layout
        public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
        public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
        public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
        public static final double highConeZ = Units.inchesToMeters(46.0);
        public static final double midConeZ = Units.inchesToMeters(34.0);

        // Translations (all nodes in the same column/row have the same X/Y coordinate)
        public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
        public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
        public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
        public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
        public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];

        static {
            for (int i = 0; i < nodeRowCount; i++) {
                boolean isCube = i == 1 || i == 4 || i == 7;
                lowTranslations[i] = new Translation2d(lowX, nodeFirstY + nodeSeparationY * i);
                midTranslations[i] = new Translation2d(midX, nodeFirstY + nodeSeparationY * i);
                mid3dTranslations[i] = new Translation3d(midX, nodeFirstY + nodeSeparationY * i,
                    isCube ? midCubeZ : midConeZ);
                high3dTranslations[i] = new Translation3d(highX, nodeFirstY + nodeSeparationY * i,
                    isCube ? highCubeZ : highConeZ);
                highTranslations[i] = new Translation2d(highX, nodeFirstY + nodeSeparationY * i);
            }
        }

        // Complex low layout
        // shifted to account for cube vs cone rows and wide edge nodes
        public static final double complexLowXCones = outerX - Units.inchesToMeters(16.0) / 2.0;
        // Centered X under cone nodes
        public static final double complexLowXCubes = lowX;
        // Centered X under cube nodes
        public static final double complexLowOuterYOffset =
            nodeFirstY - Units.inchesToMeters(3.0) - (Units.inchesToMeters(25.75) / 2.0);

        public static final Translation2d[] complexLowTranslations = new Translation2d[] {
            new Translation2d(complexLowXCones, nodeFirstY - complexLowOuterYOffset),
            new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1),
            new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 2),
            new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 3),
            new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4),
            new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 5),
            new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 6),
            new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7),
            new Translation2d(complexLowXCones,
                nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset),};
    }

    /**
     * Dimensions for loading zone and substations, including the tape
     */
    public static final class LoadingZone {
        // Region dimensions
        public static final double width = Units.inchesToMeters(99.0);
        public static final double innerX = FieldConstants.fieldLength;
        public static final double midX = fieldLength - Units.inchesToMeters(132.25);
        public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
        public static final double leftY = FieldConstants.fieldWidth;
        public static final double midY = leftY - Units.inchesToMeters(50.5);
        public static final double rightY = leftY - width;
        // Start at lower left next to border with opponent community
        public static final Translation2d[] regionCorners =
            new Translation2d[] {new Translation2d(midX, rightY), new Translation2d(midX, midY),
                new Translation2d(outerX, midY), new Translation2d(outerX, leftY),
                new Translation2d(innerX, leftY), new Translation2d(innerX, rightY),};

        // Double substation dimensions
        public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
        public static final double doubleSubstationX = innerX - doubleSubstationLength;
        public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);

        // Single substation dimensions
        public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
        public static final double singleSubstationLeftX =
            FieldConstants.fieldLength - doubleSubstationLength - Units.inchesToMeters(88.77);
        public static final double singleSubstationCenterX =
            singleSubstationLeftX + (singleSubstationWidth / 2.0);
        public static final double singleSubstationRightX =
            singleSubstationLeftX + singleSubstationWidth;
        public static final Translation2d singleSubstationTranslation =
            new Translation2d(singleSubstationCenterX, leftY);

        public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
        public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
        public static final double singleSubstationCenterZ =
            singleSubstationLowZ + (singleSubstationHeight / 2.0);
        public static final double singleSubstationHighZ =
            singleSubstationLowZ + singleSubstationHeight;
    }

    /**
     * Locations of staged game pieces
     */
    public static final class StagingLocations {
        public static final double centerOffsetX = Units.inchesToMeters(47.36);
        public static final double positionX = fieldLength / 2.0 - Units.inchesToMeters(47.36);
        public static final double firstY = Units.inchesToMeters(36.19);
        public static final double separationY = Units.inchesToMeters(48.0);
        public static final Translation2d[] translations = new Translation2d[4];

        static {
            for (int i = 0; i < translations.length; i++) {
                translations[i] = new Translation2d(positionX, firstY + (i * separationY));
            }
        }
    }

    /**
     * AprilTag locations (do not flip for red alliance)
     */
    public static final Map<Integer, Pose3d> aprilTags = Map.of(1,
        new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI)),
        2,
        new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI)),
        3,
        new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19),
            Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI)),
        4,
        new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, Math.PI)),
        5,
        new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38), new Rotation3d()),
        6,
        new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19),
            Units.inchesToMeters(18.22), new Rotation3d()),
        7,
        new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22), new Rotation3d()),
        8, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22), new Rotation3d()));

    /**
     * Flips a translation to the correct side of the field based on the current alliance color. By
     * default, all translations and poses in {@link FieldConstants} are stored with the origin at
     * the rightmost point on the BLUE ALLIANCE wall.
     */
    public static Translation2d allianceFlip(Translation2d translation) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return new Translation2d(fieldLength - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    /**
     * Flips a pose to the correct side of the field based on the current alliance color. By
     * default, all translations and poses in {@link FieldConstants} are stored with the origin at
     * the rightmost point on the BLUE ALLIANCE wall.
     *
     * @param pose
     * @return Pose2d flipped to Red Alliance
     */
    public static Pose2d allianceFlip(Pose2d pose) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return new Pose2d(fieldLength - pose.getX(), pose.getY(),
                new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            return pose;
        }
    }

    /**
     * elementwise maximum
     *
     * @param x
     * @return Max Translation
     */
    private static Translation2d max(Translation2d... x) {
        Translation2d r = x[0];
        for (int i = 1; i < x.length; i++) {
            if (x[i].getX() > r.getX()) {
                r = new Translation2d(x[i].getX(), r.getY());
            }
            if (x[i].getY() > r.getY()) {
                r = new Translation2d(r.getX(), x[i].getY());
            }
        }
        return r;
    }

    /**
     * Vector dot product for Translation2d types
     *
     * @param a
     * @param b
     * @return Dot
     */
    private static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    /**
     * https://www.youtube.com/watch?v=PMltMdi1Wzg
     *
     * @param p
     * @param p0
     * @param p1
     * @return Line SDF
     */
    private static double lineSdf(Translation2d p, Translation2d p0, Translation2d p1) {
        Translation2d pa = p.minus(p0);
        Translation2d ba = p1.minus(p0);
        double h = MathUtil.clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
        return pa.minus(ba.times(h)).getNorm();
    }

    /**
     * https://www.youtube.com/watch?v=62-pRVZuS5c
     *
     * @param P
     * @param p0
     * @param p1
     * @return Box SDF
     */
    private static double boxSdf(Translation2d P, Translation2d p0, Translation2d p1) {
        Translation2d min =
            new Translation2d(Math.min(p0.getX(), p1.getX()), Math.min(p0.getY(), p1.getY()));
        Translation2d max =
            new Translation2d(Math.max(p0.getX(), p1.getX()), Math.max(p0.getY(), p1.getY()));
        Translation2d b = max.minus(min).times(0.5);
        Translation2d p = P.minus(b).minus(min);
        Translation2d d = new Translation2d(Math.abs(p.getX()), Math.abs(p.getY())).minus(b);
        return max(d, new Translation2d()).getNorm() + Math.min(Math.max(d.getX(), d.getY()), 0.0);
    }

    /**
     * find minimum distance given a set of distances (variadic min)
     *
     * @param x
     * @return UNion SDF
     */
    private static double unionSdf(double... x) {
        double y = x[0];
        for (int i = 1; i < x.length; i++) {
            if (x[i] < y) {
                y = x[i];
            }
        }
        return y;
    }

    /**
     * Find the minimum distance from a circular robot to an obstacle on the field.
     *
     * @param p
     * @param radius
     * @return field SDF
     */
    public static double fieldSdf(Translation2d p, double radius) {
        return fieldSdf(p) - radius;
    }

    /**
     * Find the minimum distance from a point to an obstacle on the field.
     */
    public static double fieldSdf(Translation2d p) {
        Translation2d q = new Translation2d(fieldLength - p.getX(), p.getY());
        Translation2d gridCorner = new Translation2d(Grids.outerX, Community.leftY);

        double inField =
            -boxSdf(p, new Translation2d(), new Translation2d(fieldLength, fieldWidth));

        double blueGrid = boxSdf(p, new Translation2d(), gridCorner);
        double blueRail = lineSdf(p, gridCorner, Community.regionCorners[2]);
        double blueRamp =
            boxSdf(p, Community.chargingStationCorners[0], Community.chargingStationCorners[3]);
        double redGrid = boxSdf(q, new Translation2d(), gridCorner);
        double redRail = lineSdf(q, gridCorner, Community.regionCorners[2]);
        double redRamp =
            boxSdf(q, Community.chargingStationCorners[0], Community.chargingStationCorners[3]);

        return unionSdf(inField, blueGrid, blueRail, blueRamp, redGrid, redRail, redRamp);
    }

}
