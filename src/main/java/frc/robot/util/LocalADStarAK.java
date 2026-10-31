package frc.robot.util;


import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// NOTE: This file is available at
// https://gist.github.com/mjansen4857/a8024b55eb427184dbd10ae8923bd57d
public class LocalADStarAK implements Pathfinder {

    // STUDENT MAP: PATHPLANNER PATH-SEARCH ADAPTER
    // PURPOSE: Connects PathPlanner pathfinding with AdvantageKit recording and replay.
    // LOOK FOR: A path is a sequence of points the robot might follow; generating a path is not the same as driving it.
    private final ADStarIO pathfindingInputs = new ADStarIO();

    /**
     * Get if a new path has been calculated since the last time a path was retrieved
     *
     * @return True if a new path is available
     */
    @Override
    public boolean isNewPathAvailable() {
        if (!Logger.hasReplaySource()) {
            pathfindingInputs.updateIsNewPathAvailable();
        }

        Logger.processInputs("LocalADStarAK", pathfindingInputs);

        return pathfindingInputs.isNewPathAvailable;
    }

    /**
     * Get the most recently calculated path
     *
     * @param constraints The path constraints to use when creating the path
     * @param goalEndState The goal end state to use when creating the path
     * @return The PathPlannerPath created from the points calculated by the pathfinder
     */
    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        if (!Logger.hasReplaySource()) {
            pathfindingInputs.updateCurrentPathPoints(constraints, goalEndState);
        }

        Logger.processInputs("LocalADStarAK", pathfindingInputs);

        if (pathfindingInputs.calculatedPathPoints.isEmpty()) {
            return null;
        }

        return PathPlannerPath.fromPathPoints(pathfindingInputs.calculatedPathPoints, constraints, goalEndState);
    }

    /**
     * Set the start position to pathfind from
     *
     * @param startPosition Start position on the field. If this is within an obstacle it will be moved to the nearest
     *     non-obstacle node.
     */
    @Override
    public void setStartPosition(Translation2d startPosition) {
        if (!Logger.hasReplaySource()) {
            pathfindingInputs.pathfindingAlgorithm.setStartPosition(startPosition);
        }
    }

    /**
     * Set the goal position to pathfind to
     *
     * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved to the nearest
     *     non-obstacle node.
     */
    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        if (!Logger.hasReplaySource()) {
            pathfindingInputs.pathfindingAlgorithm.setGoalPosition(goalPosition);
        }
    }

    /**
     * Set the dynamic obstacles that should be avoided while pathfinding.
     *
     * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents opposite corners
     *     of a bounding box.
     * @param currentRobotPos The current position of the robot. This is needed to change the start position of the path
     *     to properly avoid obstacles
     */
    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
        if (!Logger.hasReplaySource()) {
            pathfindingInputs.pathfindingAlgorithm.setDynamicObstacles(obs, currentRobotPos);
        }
    }

    private static class ADStarIO implements LoggableInputs {
        public LocalADStar pathfindingAlgorithm = new LocalADStar();
        public boolean isNewPathAvailable = false;
        public List<PathPoint> calculatedPathPoints = Collections.emptyList();

        // READ THIS: Write this helper’s inputs to the telemetry log.
        @Override
        public void toLog(LogTable table) {
            table.put("IsNewPathAvailable", isNewPathAvailable);

            double[] pointsLogged = new double[calculatedPathPoints.size() * 2];
            int idx = 0;
            for (PathPoint point : calculatedPathPoints) {
                pointsLogged[idx] = point.position.getX();
                pointsLogged[idx + 1] = point.position.getY();
                idx += 2;
            }

            table.put("CurrentPathPoints", pointsLogged);
        }

        // READ THIS: Read this helper’s inputs back from recorded telemetry.
        @Override
        public void fromLog(LogTable table) {
            isNewPathAvailable = table.get("IsNewPathAvailable", false);

            double[] pointsLogged = table.get("CurrentPathPoints", new double[0]);

            List<PathPoint> pathPoints = new ArrayList<>();
            for (int i = 0; i < pointsLogged.length; i += 2) {
                pathPoints.add(new PathPoint(new Translation2d(pointsLogged[i], pointsLogged[i + 1]), null));
            }

            calculatedPathPoints = pathPoints;
        }

        // READ THIS: Refresh the flag that tells PathPlanner a new route was calculated.
        public void updateIsNewPathAvailable() {
            isNewPathAvailable = pathfindingAlgorithm.isNewPathAvailable();
        }

        // READ THIS: Refresh the route points read from the pathfinding algorithm.
        public void updateCurrentPathPoints(PathConstraints constraints, GoalEndState goalEndState) {
            PathPlannerPath currentPath = pathfindingAlgorithm.getCurrentPath(constraints, goalEndState);

            if (currentPath != null) {
                calculatedPathPoints = currentPath.getAllPathPoints();
            } else {
                calculatedPathPoints = Collections.emptyList();
            }
        }
    }
}
