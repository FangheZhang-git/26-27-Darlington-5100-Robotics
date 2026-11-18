package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.LynxQuadratureEncoderGroup;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

import java.util.Collections;
import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf = hardwareMap -> {
            MecanumDrive md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) md.localizer;
            List<EncoderGroup> encoderGroups = Arrays.asList(new LynxQuadratureEncoderGroup(
                    hardwareMap.getAll(LynxModule.class), Arrays.asList(dl.par, dl.perp)));
            return new DriveView(
                    DriveType.MECANUM,
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
                    encoderGroups,
                    Arrays.asList(md.leftFront, md.leftBack),
                    Arrays.asList(md.rightFront, md.rightBack),
                    Collections.emptyList(), Collections.emptyList(),
                    Arrays.asList(new EncoderRef(0, 0)),
                    Arrays.asList(new EncoderRef(0, 1)),
                    md.lazyImu,
                    md.voltageSensor,
                    () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                            MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick),
                    0);
        };

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);


        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
