# MeepMeep desktop preview

This standalone Java module uses the [Road Runner 1.0 MeepMeep fork](https://github.com/acmerobotics/MeepMeep)
and does not depend on TeamCode or deploy to the robot.

## Run

Sync Gradle in Android Studio, open
`src/main/java/org/firstinspires/ftc/meepmeeptesting/MeepMeepTesting.java`, and run
its `main()` method using the gutter Run button. Choose this desktop application
instead of the TeamCode Android run configuration.

Alternatively, from the repository root with JDK 17 or newer configured:

```sh
./gradlew :MeepMeepTesting:run
```

Compile without opening a window:

```sh
./gradlew :MeepMeepTesting:build
```

## Edit the preview

The initial preview copies the starter path: start at `(0, 0, 0)`, move to
`(24, 0)`, wait one second, and return to `(0, 0)`. Edit the action builder in
this module to try other paths. Positions are inches; angles are radians.
The background is the user-supplied BIOBUZZ image bundled at
`src/main/resources/fields/biobuzz.png`. It keeps its original orientation and
fills the square field without cropping. It loads from the classpath, so the
run configuration does not need a special working directory. Replace that PNG
to change the background. The image is a visual reference, not collision geometry;
the starter path is unchanged and may overlap field structures.

Constraints and the 18-by-18-inch robot size are preview settings. Update these
to match your measured robot. The preview uses symmetric acceleration, while
the robot drive currently has separate acceleration and deceleration limits;
simulation timing is approximate. It does not simulate mechanisms or traction.

Paths are independent copies: changes here do not update autonomous or tuning
code, and changes to robot paths are not automatically reflected here.
