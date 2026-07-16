package org.bobcatrobotics.Hardware.LedControllers;

/**
 * Represents an LED strip segment.
 * <p>
 * Contains the total number of LEDs and the start/end indices for a specific
 * segment of the strip. Used by LED controllers to target portions of the strip.
 * </p>
 */
public class LedStrip {

    /** Total number of LEDs in the strip */
    public final int length;

    /** Starting index of this LED segment (inclusive) */
    public final int startIndex;

    /** Ending index of this LED segment (exclusive) */
    public final int endIndex;

    /**
     * Constructs a new LED strip segment.
     *
     * @param length the total number of LEDs in the strip
     * @param start  the starting index of this segment (inclusive)
     * @param end    the ending index of this segment (exclusive)
     * @throws IllegalArgumentException if indices are invalid (start less than 0, end greater than length,
     *                                  or end less than or equal too start)
     */
    public LedStrip(int length, int start, int end) {
        this.length = length;
        if (start < 0 || end - start <= 0 || end > length || start > length) {
            throw new IllegalArgumentException("Invalid start or end index for LED strip segment");
        }
        this.startIndex = start;
        this.endIndex = end;
    }
}
