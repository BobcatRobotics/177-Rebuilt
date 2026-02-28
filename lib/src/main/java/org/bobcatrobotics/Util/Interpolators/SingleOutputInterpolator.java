package org.bobcatrobotics.Util.Interpolators;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * High-performance interpolator mapping:
 *
 *     distance -> (oneSpeed, twoSpeed, thirdSpeed)
 *
 * Designed for real-time shooter loops.
 */
public final class SingleOutputInterpolator implements InterpolatorIO{

    public static final class Speeds {
        public final double one;

        private Speeds(double one) {
            this.one = one;
        }

        public List<Double> getAsList(){
            List<Double> output = new ArrayList<>();
            output.add(one);
            return output;
        }
    }

    private final double[] distances;

    private final double[] oneSpeeds;

    // Precomputed slopes (per segment)
    private final double[] oneSlopes;

    private final boolean allowExtrapolation;

    // Reused output object (zero allocation)
    private final Speeds reusableSpeeds = new Speeds(0);

    public SingleOutputInterpolator(
            double[] distances,
            double[] oneSpeeds,
            boolean allowExtrapolation) {

        if (distances.length < 2 ||
            distances.length != oneSpeeds.length) {
            throw new IllegalArgumentException("All arrays must match and contain at least 2 points.");
        }

        this.distances = distances;
        this.oneSpeeds = oneSpeeds;
        this.allowExtrapolation = allowExtrapolation;

        int segments = distances.length - 1;

        this.oneSlopes = new double[segments];

        // Precompute slopes once (no division during runtime)
        for (int i = 0; i < segments; i++) {
            double dx = distances[i + 1] - distances[i];

            oneSlopes[i] = (oneSpeeds[i + 1] - oneSpeeds[i]) / dx;
        }
    }
        
    public List<Double> getAsList(double distance){
        Speeds speeds = get(distance);
        return speeds.getAsList();
    }
    
    /**
     * Gets interpolated (or extrapolated) speeds.
     *
     * No heap allocations occur in this method.
     */
    private Speeds get(double distance) {

        int index = Arrays.binarySearch(distances, distance);

        if (index >= 0) {
            return new Speeds(
                oneSpeeds[index]
            );
        }

        int insertion = -index - 1;

        if (insertion == 0) {
            if (!allowExtrapolation) {
                return new Speeds(oneSpeeds[0]);
            }
            return interpolateSegment(0, distance);
        }

        if (insertion >= distances.length) {
            if (!allowExtrapolation) {
                int last = distances.length - 1;
                return new Speeds(oneSpeeds[last]);
            }
            return interpolateSegment(distances.length - 2, distance);
        }

        return interpolateSegment(insertion - 1, distance);
    }

    private Speeds interpolateSegment(int segment, double x) {

        double baseX = distances[segment];
        double dx = x - baseX;

        double top = oneSpeeds[segment] + oneSlopes[segment] * dx;

        return new Speeds(top);
    }
}