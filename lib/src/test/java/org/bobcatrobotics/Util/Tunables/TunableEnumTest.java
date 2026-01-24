package org.bobcatrobotics.Util.Tunables;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TunableEnumTest  extends TunableTestBase {
    enum TestMode {
        A, B, C
    }

    @Test
    void enumResetsCorrectly() {
        // TunableEnum<TestMode> mode = Tunables.enumValue("Test/Mode", TestMode.B, TestMode.class);

        // assertEquals(TestMode.B, mode.get());

        // nt.getTable("Tunable").getEntry("Test/Mode").setString("C");
        // assertEquals(TestMode.C, mode.get());

        // mode.resetToDefault();
        // assertEquals(TestMode.B, mode.get());
    }
}
