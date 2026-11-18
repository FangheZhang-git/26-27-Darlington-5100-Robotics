package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;
import javax.imageio.ImageIO;

/** Desktop-only path preview; does not load FTC hardware or robot OpModes. */
public final class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Preview values only: inches, seconds, and radians. Adjust after tuning.
        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.PI, Math.PI, 13.483088305078)
                .setDimensions(18, 18)
                .build();

        // Independent copy of RoadRunnerAuto's starter path; edit previews here.
        robot.runAction(robot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .strafeTo(new Vector2d(24, 0))
                .waitSeconds(1)
                .strafeTo(new Vector2d(0, 0))
                .build());

        meepMeep.setBackground(loadFieldBackground())
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(robot)
                .start();
    }

    private static BufferedImage loadFieldBackground() {
        // Classpath resource works from Android Studio, Gradle, and packaged builds.
        try (InputStream stream = MeepMeepTesting.class.getResourceAsStream("/fields/biobuzz.png")) {
            if (stream == null) {
                throw new IllegalStateException("Missing field background: /fields/biobuzz.png");
            }
            BufferedImage image = ImageIO.read(stream);
            if (image == null) {
                throw new IllegalStateException("Cannot decode BIOBUZZ field background");
            }
            return image;
        } catch (IOException e) {
            throw new IllegalStateException("Cannot load BIOBUZZ field background", e);
        }
    }
}
