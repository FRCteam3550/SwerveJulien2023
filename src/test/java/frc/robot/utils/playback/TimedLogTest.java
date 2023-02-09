package frc.robot.utils.playback;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.AfterAll;
import static org.junit.jupiter.api.Assertions.*;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;

public class TimedLogTest {
    private static final String name = "mouvementAuto1";
    @AfterAll
    static void cleanup() throws IOException {
        // À la fin des tests, on supprime les fichiers que l'on a créé.
        Files
            .walk(Path.of(Filesystem.getDeployDirectory().getAbsolutePath()))
            .filter((f) -> f.getFileName().toString().startsWith(name) && f.toString().endsWith(".csv"))
            .forEach((f) -> f.toFile().delete());
    }
    
    @Test
    void canRecordAndPlayData() throws InterruptedException {
        var recorder = TimedLog.startRecording(name);
        recorder.recordLogEntry(0.1, 0.2);    
        Thread.sleep(10);    
        recorder.recordLogEntry(0.3, 0.4);
        Thread.sleep(20);    
        recorder.recordLogEntry(0, 0);
        recorder.save();

        var reader = TimedLog.startReadingLast(name);
        var data1 = reader.readLogEntry();
        Thread.sleep(25);    
        var data2 = reader.readLogEntry();
        Thread.sleep(20);
        var data3 = reader.readLogEntry();

        assertArrayEquals(new double[]{0.1, 0.2}, data1.get());
        assertArrayEquals(new double[]{0.3, 0.4}, data2.get());
        assertTrue(data3.isEmpty());
    }
}
