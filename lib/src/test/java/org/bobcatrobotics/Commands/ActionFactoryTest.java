package org.bobcatrobotics.Commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.concurrent.atomic.AtomicBoolean;

class ActionFactoryTest {

    ActionFactory factory = new ActionFactory();

    // Dummy subsystem implementation
    static class DummySubsystem implements Subsystem {
    }

    @Test
    void testSingleActionNoSubsystem() {
        AtomicBoolean runFlag = new AtomicBoolean(false);

        Command command = factory.singleAction("singleActionCommandWithoutSubsystem",
                () -> runFlag.set(true));

        assertEquals("singleActionCommandWithoutSubsystem", command.getName());
        command.execute();
        assertTrue(!runFlag.get());

        command.cancel();
        command.end(true);
    }

    @Test
    void testSingleActionWithSubsystem() {
        DummySubsystem subsystem = new DummySubsystem();
        AtomicBoolean runFlag = new AtomicBoolean(false);

        Command command = factory.singleAction("singleActionCommandWithSubsystem",
                () -> runFlag.set(true), subsystem);

        assertEquals("singleActionCommandWithSubsystem", command.getName());
        assertTrue(command.getRequirements().contains(subsystem));
        command.execute();
        assertTrue(!runFlag.get());

        command.cancel();
        command.end(true);
    }

    @Test
    void testContinuousActionWithoutSubsystem() {
        AtomicBoolean runFlag = new AtomicBoolean(false);
        AtomicBoolean interruptedFlag = new AtomicBoolean(false);

        Command command = factory.continuousAction("ContCmdWithoutSubsystem",
                () -> runFlag.set(true), () -> interruptedFlag.set(true));

        assertEquals("ContCmdWithoutSubsystem", command.getName());
        command.execute();
        assertTrue(runFlag.get());

        command.cancel();
        command.end(true);
        assertTrue(interruptedFlag.get());
    }

    @Test
    void testContinuousActionWithSubsystem() {
        DummySubsystem subsystem = new DummySubsystem();
        AtomicBoolean runFlag = new AtomicBoolean(false);
        AtomicBoolean interruptedFlag = new AtomicBoolean(false);

        Command command = factory.continuousAction("ContCmdWithSubsystem", () -> runFlag.set(true),
                () -> interruptedFlag.set(true), subsystem);

        assertEquals("ContCmdWithSubsystem", command.getName());
        assertTrue(command.getRequirements().contains(subsystem));
        command.execute();
        assertTrue(runFlag.get());

        command.cancel();
        command.end(true);
        assertTrue(interruptedFlag.get());
    }

    @Test
    void testAddTimeout() {
        Command baseCommand = factory.singleAction("TimeoutCmd", () -> {
        });
        Command timedCommand = factory.addTimeout(baseCommand, 5.0);
        assertEquals(factory.hasTimeout, true);
    }

    @Test
    void testAddCondition() {
        AtomicBoolean conditionMet = new AtomicBoolean(false);
        Command baseCommand = factory.singleAction("ConditionalCmd", () -> {
        });
        Command conditionalCommand = factory.addCondition(baseCommand, () -> conditionMet.get());
        assertEquals(factory.hasCondition, true);
    }

    @Test
    void testLogCommandBasic() {
        // Since Logger is static, we can't check outputs without modifying it
        DummySubsystem subsystem = new DummySubsystem();
        Command action = factory.singleAction("ActionToLog", () -> {
        });

        Command logged = factory.logCommand(action, "TestKey");

        assertEquals(action.getName(), logged.getName());
    }
}
