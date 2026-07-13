package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import edu.wpi.first.math.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Localizer;
import org.marsroboticsassociation.controllib.localization.vision.HypothesisBankLocalizer;
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolver;
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolverConfig;
import org.marsroboticsassociation.controllib.localization.vision.VisionSource;

import java.util.function.LongSupplier;

/**
 * Road Runner {@link Localizer} adapter around the library's pure {@link HypothesisBankLocalizer}.
 * This is the thin FTC/Road Runner glue: it owns the odometry {@link Localizer} (e.g. a Pinpoint),
 * the vision {@link VisionSource}, the clock, and the FtcDashboard {@code @Config} tuning, and each
 * loop it reads odometry, converts Road Runner ↔ WPILib {@code Pose2d}/{@code PoseVelocity2d}, and
 * delegates the estimation to the library core.
 *
 * <p>All the multi-hypothesis estimation, gating, and latency compensation lives in ControlLib and
 * is desktop-testable; nothing library-specific leaks into it. Swap this adapter for a Pedro
 * Pathing one (same core, different pose conversions) to use the localizer with another drive
 * library.
 */
public final class HypothesisBankRoadRunnerLocalizer implements Localizer {

    /** FtcDashboard-tunable configuration (live). */
    @Config
    public static class Params {
        /** The pure library core's tuning (bank + commit + latency + blur gate). */
        public HypothesisBankLocalizer.Params core = new HypothesisBankLocalizer.Params();

        /** AprilTag black-square side length (metres) for the ambiguity PnP re-solve. */
        public double tagSizeMeters = 0.1651;

        /**
         * Minimum tag image area (percent of frame) for a tag to qualify as the ambiguity anchor; 0
         * disables the area filter (any tag qualifies).
         */
        public double ambiguityMinTagAreaPct = 0.0;
    }

    public static Params PARAMS = new Params();

    private final Localizer odometry;
    private final VisionSource source;
    private final Telemetry telemetry;
    private final LongSupplier clock;
    private final HypothesisBankLocalizer core;

    /**
     * Constructs the adapter with the default {@link VisionPoseSolverConfig} solver and the {@link
     * System#nanoTime()} clock.
     *
     * @param odometry the odometry localizer (the rigid backbone; its raw pose need not be
     *     field-aligned — the bank maps it to the field)
     * @param source the vision-acquisition source (e.g. {@link LimelightVisionSource})
     * @param startPose the starting field pose to seed (Road Runner)
     * @param telemetry for status logging; may be null
     */
    public HypothesisBankRoadRunnerLocalizer(
            Localizer odometry,
            VisionSource source,
            com.acmerobotics.roadrunner.Pose2d startPose,
            Telemetry telemetry) {
        this(
                odometry,
                source,
                new VisionPoseSolverConfig().solver(),
                startPose,
                telemetry,
                System::nanoTime);
    }

    public HypothesisBankRoadRunnerLocalizer(
            Localizer odometry,
            VisionSource source,
            VisionPoseSolver solver,
            com.acmerobotics.roadrunner.Pose2d startPose,
            Telemetry telemetry,
            LongSupplier clock) {
        this.odometry = odometry;
        this.source = source;
        this.telemetry = telemetry;
        this.clock = clock;
        this.core = new HypothesisBankLocalizer(PARAMS.core, solver);
        if (startPose != null) {
            core.setPose(toWpi(startPose), toWpi(odometry.getPose()));
        }
    }

    @Override
    public void setPose(com.acmerobotics.roadrunner.Pose2d pose) {
        core.setPose(toWpi(pose), toWpi(odometry.getPose()));
    }

    @Override
    public com.acmerobotics.roadrunner.Pose2d getPose() {
        return toRr(core.getPose(toWpi(odometry.getPose())));
    }

    @Override
    public PoseVelocity2d update() {
        PoseVelocity2d vel = odometry.update();
        com.acmerobotics.roadrunner.Pose2d odoRr = odometry.getPose();

        // Keep the vision source's robot-orientation seed current (cheap; MT1 doesn't need it, but
        // it keeps MT2 usable if ever reintroduced as a check).
        source.updateRobotOrientation(Math.toDegrees(getPose().heading.toDouble()));

        core.update(clock.getAsLong(), toWpi(odoRr), vel.angVel, source.latest());

        if (telemetry != null) {
            telemetry.addData("bank_committed", core.isCommitted());
            telemetry.addData("bank_domWeight", String.format("%.2f", core.dominantWeight()));
            telemetry.addData("bank_size", core.bank().size());
        }
        return vel;
    }

    /** Whether the bank has committed (gate auto-aim / trusted-pose cues on this). */
    public boolean isCommitted() {
        return core.isCommitted();
    }

    /** Commitment level: dominant-hypothesis weight in {@code (0,1]}. */
    public double dominantWeight() {
        return core.dominantWeight();
    }

    /**
     * The raw (uncorrected) odometry pose this loop, Road Runner frame — for logging/diagnostics.
     */
    public com.acmerobotics.roadrunner.Pose2d getOdometryPose() {
        return odometry.getPose();
    }

    /** The pure library core, for diagnostic access (see {@link BankLocalizerCsvLogger}). */
    public HypothesisBankLocalizer core() {
        return core;
    }

    /**
     * The vision source, for its calibration (intrinsics/distortion) when logging the replay row.
     */
    public VisionSource visionSource() {
        return source;
    }

    private static edu.wpi.first.math.geometry.Pose2d toWpi(com.acmerobotics.roadrunner.Pose2d p) {
        return new edu.wpi.first.math.geometry.Pose2d(
                p.position.x, p.position.y, new Rotation2d(p.heading.toDouble()));
    }

    private static com.acmerobotics.roadrunner.Pose2d toRr(edu.wpi.first.math.geometry.Pose2d p) {
        return new com.acmerobotics.roadrunner.Pose2d(
                p.getX(), p.getY(), p.getRotation().getRadians());
    }
}
