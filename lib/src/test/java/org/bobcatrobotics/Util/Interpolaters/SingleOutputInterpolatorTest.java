package org.bobcatrobotics.Util.Interpolaters;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

import org.bobcatrobotics.Util.Interpolators.SingleOutputInterpolator;

class SingleOutputInterpolatorTest  {

  @Test
  void testInterpolation() {
      double[] distances = new double[] { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };
      double[] oneSpeeds = new double[15];
      for (int i = 0; i < 15; i++) {
        oneSpeeds[i] = i * 0.25;
      }
      boolean allowExtrapolation = true;
      SingleOutputInterpolator data = new SingleOutputInterpolator(distances, oneSpeeds, allowExtrapolation);

      double speed = data.getAsList(0.5).get(0);
      assertEquals(0.125, speed);

      double speed2 = data.getAsList(0.25).get(0);
      assertEquals(0.0625, speed2);
  }
}
