package org.team100.lib.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum FieldPoint2024 {
    NOTE1, NOTE2, NOTE3, NOTE4, NOTE5, NOTE6, NOTE7, NOTE8,
    NOTE9, NOTE10, NOTE11,
    CLOSEWINGSHOT, FARWINGSHOT, STAGESHOT,
    CENTRALSTAGEOPENING,
    FARSTAGEADJACENT, FARSTAGEOPENING, DROPSHOT,
    CLOSESTAGEADJACENT, STARTSUBWOOFER, DRIVETONOTEHANDOFF, CITRUSMID,
    CITRUSEND, CITRUSBEGIN,
    COMPLEMENTBEGIN, COMPLEMENTSHOOT, COMPLEMENTSHOOT2;

    public static Translation2d[] allNotes(Alliance alliance) {
        return new Translation2d[] {
                getTranslation(alliance, FieldPoint2024.NOTE1),
                getTranslation(alliance, FieldPoint2024.NOTE2),
                getTranslation(alliance, FieldPoint2024.NOTE3),
                getTranslation(alliance, FieldPoint2024.NOTE4),
                getTranslation(alliance, FieldPoint2024.NOTE5),
                getTranslation(alliance, FieldPoint2024.NOTE6),
                getTranslation(alliance, FieldPoint2024.NOTE7),
                getTranslation(alliance, FieldPoint2024.NOTE8),
                getTranslation(alliance, FieldPoint2024.NOTE9),
                getTranslation(alliance, FieldPoint2024.NOTE10),
                getTranslation(alliance, FieldPoint2024.NOTE11)
        };
    }

    public static Translation2d getTranslation(Alliance m_alliance, FieldPoint2024 point) {
        return switch (point) {
            case NOTE1 -> forAlliance(new Translation2d(2.8956, 7.0061), m_alliance);
            case NOTE2 -> forAlliance(new Translation2d(2.8956, 5.5583), m_alliance);
            case NOTE3 -> forAlliance(new Translation2d(2.8956, 4.1105), m_alliance);// 2.6956, 4.2105
            case NOTE4 -> forAlliance(new Translation2d(8.271, 0.75), m_alliance);
            case NOTE5 -> forAlliance(new Translation2d(8.271, 2.4341), m_alliance);
            case NOTE6 -> forAlliance(new Translation2d(8.271, 4.1105), m_alliance);
            case NOTE7 -> forAlliance(new Translation2d(8.271, 5.7869), m_alliance);
            case NOTE8 -> forAlliance(new Translation2d(8.2, 7.6), m_alliance);// WAS 7.4 //8.4 fudge
            case CLOSEWINGSHOT -> forAlliance(new Translation2d(3, 6.4), m_alliance);
            case FARWINGSHOT -> forAlliance(new Translation2d(4, 1.5), m_alliance);
            case STAGESHOT -> forAlliance(new Translation2d(4.25, 5), m_alliance);
            case CENTRALSTAGEOPENING -> forAlliance(new Translation2d(5.87248, 4.1105), m_alliance);
            case FARSTAGEOPENING -> forAlliance(new Translation2d(4.3, 3.3), m_alliance);
            case FARSTAGEADJACENT -> forAlliance(new Translation2d(5.87248, 1.9), m_alliance);
            case CLOSESTAGEADJACENT -> forAlliance(new Translation2d(5.87248, 6.45), m_alliance);
            case DROPSHOT -> forAlliance(new Translation2d(.5, 1.8), m_alliance);
            case STARTSUBWOOFER -> forAlliance(new Translation2d(1.38, 5.566847), m_alliance);
            case DRIVETONOTEHANDOFF -> forAlliance(new Translation2d(4.9, 7.5), m_alliance);
            case CITRUSMID -> forAlliance(new Translation2d(3.269011, 1.306543), m_alliance);
            case CITRUSEND -> forAlliance(new Translation2d(6.964483, 1.476407), m_alliance);
            case CITRUSBEGIN -> forAlliance(new Translation2d(0.865358, 4.215958), m_alliance);
            case COMPLEMENTBEGIN -> forRedAlliance(new Translation2d(0.663879, 3.917024), m_alliance);
            case COMPLEMENTSHOOT -> forRedAlliance(new Translation2d(3.47, 5.06), m_alliance);// 3.183978, 4.8
            case COMPLEMENTSHOOT2 -> forRedAlliance(new Translation2d(3.94, 2.94), m_alliance);
            default -> new Translation2d();
        };
    }

    public static Translation2d forAlliance(Translation2d translation, Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return translation;
        }
        return new Translation2d(translation.getX(), 8.221 - translation.getY());
    }

    public static Translation2d forRedAlliance(Translation2d translation, Alliance alliance) {
        if (alliance == Alliance.Red) {
            return translation;
        }
        return new Translation2d(translation.getX(), 8.221 - translation.getY());
    }
}