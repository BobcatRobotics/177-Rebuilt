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
public final class TripleOutputInterpolator implements InterpolatorIO{

    public static final class Speeds {
        public final double one;
        public final double two;
        public final double three;

        private Speeds(double one, double two, double three) {
            this.one = one;
            this.two = two;
            this.three = three;
        }

        public List<Double> getAsList(){
            List<Double> output = new ArrayList<>();
            output.add(one);
            output.add(two);
            output.add(three);
            return output;
        }
    }

    private final double[] distances;

    private final double[] oneSpeeds;
    private final double[] twoSpeeds;
    private final double[] threeSpeeds;

    // Precomputed slopes (per segment)
    private final double[] oneSlopes;
    private final double[] twoSlopes;
    private final double[] threeSlopes;

    private final boolean allowExtrapolation;

    // Reused output object (zero allocation)
    private final Speeds reusableSpeeds = new Speeds(0, 0, 0);

    public TripleOutputInterpolator(
            double[] distances,
            double[] oneSpeeds,
            double[] twoSpeeds,
            double[] threeSpeeds,
            boolean allowExtrapolation) {

        if (distances.length < 2 ||
            distances.length != oneSpeeds.length ||
            distances.length != twoSpeeds.length ||
            distances.length != threeSpeeds.length) {
            throw new IllegalArgumentException("All arrays must match and contain at least 2 points.");
        }

        this.distances = distances.clone();
        this.oneSpeeds = oneSpeeds.clone();
        this.twoSpeeds = twoSpeeds.clone();
        this.threeSpeeds = threeSpeeds.clone();
        this.allowExtrapolation = allowExtrapolation;

        int segments = distances.length - 1;

        this.oneSlopes = new double[segments];
        this.twoSlopes = new double[segments];
        this.threeSlopes = new double[segments];

        // Precompute slopes once (no division during runtime)
        for (int i = 0; i < segments; i++) {
            double dx = distances[i + 1] - distances[i];

            oneSlopes[i] = (oneSpeeds[i + 1] - oneSpeeds[i]) / dx;
            twoSlopes[i] = (twoSpeeds[i + 1] - twoSpeeds[i]) / dx;
            threeSlopes[i] = (threeSpeeds[i + 1] - threeSpeeds[i]) / dx;
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
                oneSpeeds[index],
                twoSpeeds[index],
                threeSpeeds[index]
            );
        }

        int insertion = -index - 1;

        if (insertion == 0) {
            if (!allowExtrapolation) {
                return new Speeds(oneSpeeds[0], twoSpeeds[0], threeSpeeds[0]);
            }
            return interpolateSegment(0, distance);
        }

        if (insertion >= distances.length) {
            if (!allowExtrapolation) {
                int last = distances.length - 1;
                return new Speeds(oneSpeeds[last], twoSpeeds[last], threeSpeeds[last]);
            }
            return interpolateSegment(distances.length - 2, distance);
        }

        return interpolateSegment(insertion - 1, distance);
    }

    private Speeds interpolateSegment(int segment, double x) {

        double baseX = distances[segment];
        double dx = x - baseX;

        double top = oneSpeeds[segment] + oneSlopes[segment] * dx;
        double bottom = twoSpeeds[segment] + twoSlopes[segment] * dx;
        double feeder = threeSpeeds[segment] + threeSlopes[segment] * dx;

        return new Speeds(top, bottom, feeder);
    }
}