package frc.robot.utils.playback;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Optional;
import java.util.Scanner;
import java.util.StringJoiner;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

public class TimedLog implements LogReader, LogRecorder {
    private static final SimpleDateFormat DATE_FORMAT = new SimpleDateFormat("yyyy_MM_dd_hh_mm_ss");
    private static final String DIRECTORY = Filesystem.getDeployDirectory().getAbsolutePath();
    private final String name;
    private final Timer timer = new Timer();
    private List<TimedLogEntry> entries = new ArrayList<>();
    private int currentLogEntry = 0;

    private TimedLog(String name) {
        this(name, new ArrayList<>());
    }

    private TimedLog(String name, List<TimedLogEntry> entries) {
        this.name = name;
        this.entries = entries;
        timer.reset();
        timer.start();
    }

    public static LogRecorder startRecording(String name) {
        return new TimedLog(name);
    }

    /**
     * Enregistre les infos avec le temps écoulé depuis le début de l'enregistrement.
     * Le temps écoulé commence avec le premier enregistrement d'infos.
     * @param data
     */
    public void recordLogEntry(double... data) {
        entries.add(new TimedLogEntry(timer.get(), data));
    }

    /**
     * Retourne la donnée pour le temps qui s'est écoulé depuis resetPlay(), ou rien si la lecture est terminée.
     */
    public Optional<double[]> readLogEntry() {
        var elapsedTimeSeconds = timer.get();
        while (currentLogEntry < entries.size() - 1 && elapsedTimeSeconds > entries.get(currentLogEntry + 1).elapsedTimeSeconds) {
            currentLogEntry += 1;
        }

        if (currentLogEntry < entries.size() - 1) {
            return Optional.of(entries.get(currentLogEntry).data);
        }
        return Optional.empty();
    }

    private File saveFile() {
        var fileName = String.format("%s_%s.csv", name, DATE_FORMAT.format(new Date()));
        return Path.of(DIRECTORY, fileName).toFile();
    }

    /**
     * Sauve l'enregistrement dans un fichier dont le nom est l'assemblage du nom du log et de la date.
     * Le répertoire est le répertoire de déploiement.
     */
    public void save() {
        var file = saveFile();
        try (var writer = new PrintWriter(new BufferedWriter(new FileWriter(file)))) {
            for (var entry : entries) {
                var sj = new StringJoiner(",");
                sj.add(Double.toString(entry.elapsedTimeSeconds));
                for (var data : entry.data) {
                    sj.add(Double.toString(data));
                }
                writer.println(sj);
            }

            System.out.println(String.format("Sauvé l'enregistrement dans %s", file.getAbsolutePath()));
        }
        catch(IOException ioe) {
            var msg = String.format("Impossible de sauver l'enregistrement dans %s", file.getAbsolutePath());
            DriverStation.reportError(msg, false);
            System.out.println(msg);
        }
    }

    /**
     * Charge l'enregistrement avec le nom de fichier donné, à partir du répertoire de déploiement.
     */
    public static LogReader startReading(String fileName) {
        var file = Path.of(DIRECTORY, fileName).toFile();

        try (var scanner = new Scanner(new FileReader(file))) {
            var result = new ArrayList<TimedLogEntry>();
            while (scanner.hasNextLine()) {
                var dataAsString = scanner.nextLine().split(",");
                var data = Stream.of(dataAsString).mapToDouble(Double::valueOf).toArray();
                var entry = new TimedLogEntry(
                   data[0],
                   Arrays.copyOfRange(data, 1, data.length)
                );
                result.add(entry);
            }

            var name = fileName.split("_")[0];
            System.out.println(String.format("Chargé l'enregistrement de %s", file.getAbsolutePath()));
            return new TimedLog(name, result);
        }
        catch(IOException ioe) {
            throw new RuntimeException(String.format("Impossible de lire l'enregistrement de %s", file.getAbsolutePath()), ioe);
        }
    }

    /**
     * Charge le dernier enregistrement avec le nom donné, à partir du répertoire de déploiement.
     */
    public static LogReader startReadingLast(String name) {
        var prefix = name + "_";

        try {
            var files = Files
                .walk(Path.of(DIRECTORY))
                .map((f) -> f.getFileName().toString())
                .filter((f) -> f.startsWith(prefix) && f.endsWith(".csv"))
                .sorted()
                .toList();
            if (files.size() == 0) {
                throw new IOException();
            }

            var lastFile = files.get(files.size() - 1);
            return startReading(lastFile);
        }
        catch (IOException ioe) {
            throw new RuntimeException(String.format("Impossible de trouver un enregistrement avec le nom %s dans %s", name, DIRECTORY), ioe);
        }
    }
}
