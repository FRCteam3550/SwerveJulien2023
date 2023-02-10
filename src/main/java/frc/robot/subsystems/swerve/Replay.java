package frc.robot.subsystems.swerve;

import java.util.Optional;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.playback.LogReader;
import frc.robot.utils.playback.LogRecorder;
import frc.robot.utils.playback.TimedLog;

public class Replay implements Sendable {
    private static final int X = 0;
    private static final int Y = 1;
    private static final int ROT = 2;
    private String name = "auto1";
    private LogRecorder recorder;
    private LogReader reader;

    public Replay() {
        SendableRegistry.add(this, "Replay");
        SmartDashboard.putData(this);
    }

    public boolean isRecording() {
        return recorder != null;
    }

    public void startRecording() {
        System.out.println("Start recording...");
        recorder = TimedLog.startRecording(name);
    }

    public void record(JoystickInputs inputs) {
        recorder.recordLogEntry(inputs.x, inputs.y, inputs.rot);
    }

    public void stopRecording() {
        System.out.println("Stop recording...");
        recorder.save();
        recorder = null;
    }

    public void startPlaying() {
        System.out.println("Start playing...");
        reader = TimedLog.startReadingLast(name);
    }

    public Optional<JoystickInputs> play() {
        return reader
            .readLogEntry()
            .map((data) -> new JoystickInputs(data[X], data[Y], data[ROT]));
    }

    public boolean playIsFinished() {
        return reader == null || play().isEmpty();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("is recording", this::isRecording, null);
        builder.addBooleanProperty("play is finished", this::playIsFinished, null);
        builder.addStringProperty("name", () -> name, (val) -> name = val);
    }
}
