// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;


import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or value not in
 * dashboard.
 */
@SuppressWarnings("unused")
public class LoggedTunableNumber implements DoubleSupplier {

    // STUDENT MAP: DASHBOARD-ADJUSTABLE NUMBER
    // PURPOSE: Lets a number be configured and read from the dashboard while preserving a default value.
    // LOOK FOR: Read get() to see how the current value is chosen; hasChanged() checks for updates.
    private static final String tableKey = "/Tuning";

    private final String fullDashboardKey;
    private boolean hasDefaultValue = false;
    private double defaultValue;
    private LoggedNetworkNumber networkTableEntry;
    private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public LoggedTunableNumber(String dashboardKey) {
        this.fullDashboardKey = tableKey + "/" + dashboardKey;
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue) {
        if (!hasDefaultValue) {
            hasDefaultValue = true;
            this.defaultValue = defaultValue;
            networkTableEntry = new LoggedNetworkNumber(fullDashboardKey, defaultValue);
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    // STUDENT NOTE: Return the latest dashboard value, or the default if unavailable.
    public double get() {
        if (!hasDefaultValue) {
            return 0.0;
        } else {
            return networkTableEntry.get();
        }
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple objects. Recommended
     *     approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false otherwise.
     */
    // STUDENT NOTE: Check whether this number changed since a given caller last inspected it.
    public boolean hasChanged(int id) {
        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    /**
     * Runs action if any of the tunableNumbers have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple * objects. Recommended
     *     approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable numbers have changed. Access tunable numbers in order
     *     inputted in method
     * @param tunableNumbers All tunable numbers to check
     */
    // STUDENT NOTE: Run an action when any watched tuning number changes.
    public static void ifChanged(int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
        if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
            action.accept(Arrays.stream(tunableNumbers)
                .mapToDouble(LoggedTunableNumber::get)
                .toArray());
        }
    }

    /** Runs action if any of the tunableNumbers have changed */
    // STUDENT NOTE: Run an action when any watched tuning number changes.
    public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
        ifChanged(id, values -> action.run(), tunableNumbers);
    }

    // READ THIS: Expose this value as a Java DoubleSupplier.
    @Override
    public double getAsDouble() {
        return get();
    }
}
