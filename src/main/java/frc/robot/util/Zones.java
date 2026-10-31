package frc.robot.util;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Dimensions;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.RightBump;
import frc.robot.FieldConstants.RightTrench;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

// FILE GUIDE: Named field regions: construct triggers for being inside or approaching a region.
public class Zones {

    // STUDENT MAP: AREAS ON THE FIELD
    // PURPOSE: Defines rectangles or groups of regions and tests whether the robot is inside or approaching them.
    // LOOK FOR: contains means in the region NOW; willContain predicts a future position using velocity and time.
    public interface Zone {
        Trigger contains(Supplier<Pose2d> pose);
    }

    public interface PredictiveXZone extends Zone {
        Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt);
    }

    // A BaseZone is a rectangle between two X and two Y boundaries.
    // Coordinates describe positions on the playing field in meters.
    public static class BaseZone implements Zone {
        protected final double xMin, xMax, yMin, yMax;

        // READ THIS: Define a rectangular region of the field using its boundaries.
        public BaseZone(double xMin, double xMax, double yMin, double yMax) {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
        }

        // READ THIS: Define a rectangular region of the field using its boundaries.
        public BaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
            this(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
        }

        @Override

        // STUDENT NOTE: Create a true/false condition for being inside this field zone.
        public Trigger contains(Supplier<Pose2d> poseSupplier) {
            return new Trigger(() -> this.containsPoint(poseSupplier.get().getTranslation()));
        }

        // READ THIS: Check whether a point is inside this rectangle.
        protected boolean containsPoint(Translation2d point) {
            return point.getX() >= xMin && point.getX() <= xMax && point.getY() >= yMin && point.getY() <= yMax;
        }

        // STUDENT NOTE: Return a matching area mirrored across the field length.
        public BaseZone mirroredX() {
            return new BaseZone(
                FieldConstants.fieldLength - xMax,
                FieldConstants.fieldLength - xMin,
                yMin,
                yMax);
        }

        // STUDENT NOTE: Return a matching area mirrored across the field width.
        public BaseZone mirroredY() {
            return new BaseZone(
                xMin,
                xMax,
                FieldConstants.fieldWidth - yMax,
                FieldConstants.fieldWidth - yMin);
        }

        /** list of corners, with the bottom left corner repeated at the end to form a closed loop */
        public Translation2d[] getCorners() {
            return new Translation2d[] {
                new Translation2d(xMin, yMin),
                new Translation2d(xMax, yMin),
                new Translation2d(xMax, yMax),
                new Translation2d(xMin, yMax),
                new Translation2d(xMin, yMin)
            };
        }
    }

    // Predictive zone = current zone boundaries plus a forward-motion check.
    // It uses X velocity * time to estimate where the robot may be soon.
    public static class PredictiveXBaseZone extends BaseZone implements PredictiveXZone {

        // READ THIS: Create a rectangle that can predict entry using field-relative speed.
        public PredictiveXBaseZone(double xMin, double xMax, double yMin, double yMax) {
            super(xMin, xMax, yMin, yMax);
        }

        // READ THIS: Create a rectangle that can predict entry using field-relative speed.
        public PredictiveXBaseZone(BaseZone baseZone) {
            super(baseZone.xMin, baseZone.xMax, baseZone.yMin, baseZone.yMax);
        }

        // READ THIS: Create a rectangle that can predict entry using field-relative speed.
        public PredictiveXBaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
            super(xMin, xMax, yMin, yMax);
        }

        @Override

        // STUDENT NOTE: Create a condition estimating whether the current movement will enter the zone soon.
        public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
            return new Trigger(() -> willContainPoint(pose.get().getTranslation(), fieldSpeeds.get(), dt));
        }

        // READ THIS: Project the position forward using measured field speed and check the region.
        protected boolean willContainPoint(Translation2d point, ChassisSpeeds fieldSpeeds, Time dt) {
            return (point.getY() >= yMin && point.getY() <= yMax)
                && ((point.getX() >= xMin && point.getX() <= xMax)
                || (point.getX() < xMin
                && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) >= xMin - point.getX())
                || (point.getX() > xMax
                && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) <= xMax - point.getX()));
        }

        @Override

        // STUDENT NOTE: Return a matching area mirrored across the field length.
        public PredictiveXBaseZone mirroredX() {
            return new PredictiveXBaseZone(super.mirroredX());
        }

        @Override

        // STUDENT NOTE: Return a matching area mirrored across the field width.
        public PredictiveXBaseZone mirroredY() {
            return new PredictiveXBaseZone(super.mirroredY());
        }
    }

    public static class ZoneCollection implements Zone {
        protected final Zone[] zones;

        // READ THIS: Combine several regions so one trigger can cover all of them.
        public ZoneCollection(Zone... zones) {
            this.zones = zones;
        }

        @Override

        // STUDENT NOTE: Create a true/false condition for being inside this field zone.
        public Trigger contains(Supplier<Pose2d> pose) {
            Trigger combined = new Trigger(() -> false);

            for (Zone zone : zones) {
                combined = combined.or(zone.contains(pose));
            }

            return combined;
        }
    }

    public static class PredictiveXZoneCollection extends ZoneCollection implements PredictiveXZone {

        // READ THIS: Combine several predictive regions into one look-ahead trigger.
        public PredictiveXZoneCollection(PredictiveXZone... zones) {
            super(zones);
        }

        @Override

        // STUDENT NOTE: Create a condition estimating whether the current movement will enter the zone soon.
        public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
            Trigger combined = new Trigger(() -> false);

            for (Zone zone : zones) {
                combined = combined.or(((PredictiveXZone) zone).willContain(pose, fieldSpeeds, dt));
            }

            return combined;
        }
    }

    private static final PredictiveXBaseZone BLUE_BOTTOM_TRENCH = new PredictiveXBaseZone(
        Units.Meters.of(RightTrench.width)
            .minus(Units.Meters.of(RightTrench.depth / 2))
            .minus(Dimensions.FULL_LENGTH.div(2)),
        Units.Meters.of(RightTrench.width)
            .plus(Units.Meters.of(RightTrench.depth / 2))
            .plus(Dimensions.FULL_LENGTH.div(2)),
        Meters.of(0),
        Units.Meters.of(RightTrench.width));
    private static final PredictiveXBaseZone BLUE_TOP_TRENCH = BLUE_BOTTOM_TRENCH.mirroredY();
    private static final PredictiveXBaseZone RED_BOTTOM_TRENCH = BLUE_BOTTOM_TRENCH.mirroredX();
    private static final PredictiveXBaseZone RED_TOP_TRENCH = BLUE_TOP_TRENCH.mirroredX();

    public static final PredictiveXZoneCollection TRENCH_ZONES =
        new PredictiveXZoneCollection(BLUE_BOTTOM_TRENCH, BLUE_TOP_TRENCH, RED_BOTTOM_TRENCH, RED_TOP_TRENCH);


    private static final PredictiveXBaseZone BLUE_BOTTOM_BUMP = new PredictiveXBaseZone(
        Units.Meters.of(RightBump.nearRightCorner.getX()),
        Units.Meters.of(RightBump.farLeftCorner.getX()),
        Units.Meters.of(RightBump.nearRightCorner.getY()),
        Units.Meters.of(RightBump.farLeftCorner.getY())
         );
    private static final PredictiveXBaseZone BLUE_TOP_BUMP = BLUE_BOTTOM_BUMP.mirroredY();
    private static final PredictiveXBaseZone RED_BOTTOM_BUMP = BLUE_BOTTOM_BUMP.mirroredX();
    private static final PredictiveXBaseZone RED_TOP_BUMP = BLUE_TOP_BUMP.mirroredX();

    public static final PredictiveXZoneCollection BUMP_ZONES =
        new PredictiveXZoneCollection(BLUE_BOTTOM_BUMP, BLUE_TOP_BUMP, RED_BOTTOM_BUMP, RED_TOP_BUMP);

    // READ THIS: Publish defined zones for diagnostics and field visualization.
    public static void logAllZones() {
        Logger.recordOutput("Zones/Trenches/Blue Bottom", BLUE_BOTTOM_TRENCH.getCorners());
        Logger.recordOutput("Zones/Trenches/Blue Top", BLUE_TOP_TRENCH.getCorners());
        Logger.recordOutput("Zones/Trenches/Red Bottom", RED_BOTTOM_TRENCH.getCorners());
        Logger.recordOutput("Zones/Trenches/Red Top", RED_TOP_TRENCH.getCorners());

        Logger.recordOutput("Zones/Bumps/Blue Bottom", BLUE_BOTTOM_BUMP.getCorners());
        Logger.recordOutput("Zones/Bumps/Blue Top", BLUE_TOP_BUMP.getCorners());
        Logger.recordOutput("Zones/Bumps/Red Bottom", RED_BOTTOM_BUMP.getCorners());
        Logger.recordOutput("Zones/Bumps/Red Top", RED_TOP_BUMP.getCorners());
    }
}
