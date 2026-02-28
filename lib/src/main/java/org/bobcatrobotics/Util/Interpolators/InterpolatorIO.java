package org.bobcatrobotics.Util.Interpolators;

import java.util.ArrayList;
import java.util.List;

public interface InterpolatorIO {
    /**
     * Gets interpolated (or extrapolated) speeds.
     *
     * No heap allocations occur in this method.
     */
    public default List<Double> getAsList(double distance){
        return new ArrayList<Double>();
    }
}
