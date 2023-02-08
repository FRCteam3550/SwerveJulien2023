package frc.robot.utils.playback;

public interface LogRecorder {
    /**
     * Enregistre un ensemble de données dans le journal pour le moment courant.
     */
    void recordLogEntry(double... data);
    /**
     * Sauve le journal sur fichier.
     */
    void save();
}
