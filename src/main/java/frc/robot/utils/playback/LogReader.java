package frc.robot.utils.playback;

import java.util.Optional;

public interface LogReader {
    /**
     * Retourne les données du journal pour le moment courant, ou rien si la lecture est terminée.
     */
    Optional<double[]> readLogEntry();
}
