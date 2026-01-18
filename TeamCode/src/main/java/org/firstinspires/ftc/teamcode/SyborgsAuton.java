package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Optional;
import java.util.function.Supplier;

@Autonomous
public class SyborgsAuton extends LinearOpMode {

	MecanumDrive drive;
	LimeLightAprilTag ll;
	Shooter shooter;
	PoseFilter poseFilter;
	double shootingTime = 5.7;

	double preloadShootingTime = 3.5;
	Supplier<Action> startIntakeAction = () -> new InstantAction(() -> {
		shooter.startIntake(getRuntime());
	});
	Supplier<Action> shootAction = () -> new ParallelAction(
			new SequentialAction(shooter.feedBallsAction(),
					new SleepAction(shootingTime),
					shooter.stopFeedingAction()),
			new SequentialAction(startIntakeAction.get(), new SleepAction(0.5), shooter.stopIntakeAction(), new SleepAction(0.5), startIntakeAction.get(), new SleepAction(0.5), shooter.stopIntakeAction(), new SleepAction(0.5), startIntakeAction.get(), new SleepAction(0.5), shooter.stopIntakeAction())
	);
	Supplier<Action> preloadShootAction = () -> new ParallelAction(
			new SequentialAction(shooter.feedBallsAction(),
					new SleepAction(preloadShootingTime),
					shooter.stopFeedingAction()),
			new SequentialAction(startIntakeAction.get())
	);
	Pose2d lastPoseInit = new Pose2d(0, 0, Math.toRadians(180));
	int obeliskID = -1;
	PoseMap poseMap = new IdentityPoseMap();
	boolean farStart = false;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
		ll = new LimeLightAprilTag(hardwareMap, telemetry);
		shooter = new Shooter(hardwareMap, telemetry);
		poseFilter = new PoseFilter();
		while (opModeInInit()) {
			runInitLoop();
		}
		waitForStart();
		drive.localizer.setPose(lastPoseInit);
		if (Common.alliance == Common.Alliance.Blue) {
			poseMap = pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());
		}
//		while (opModeIsActive()) {
//			drive.updatePoseEstimate();
//			TelemetryPacket packet = new TelemetryPacket();
//			Pose2d pose = drive.localizer.getPose();
//			packet.fieldOverlay().setStroke("#3F51B5"); // blue for pinpoint pose
//			Drawing.drawRobot(packet.fieldOverlay(), pose);
//			FtcDashboard.getInstance().sendTelemetryPacket(packet);
//		}
		// ensure that the correct cycle is never shot last, so it doesn't become overflow and lose some bonus points
		Action autonAction = new SequentialAction(runPreloaded(), runCycleGPP(), runCyclePPG(), leaveShootZone());
		if (obeliskID == 22) {
			autonAction = new SequentialAction(runPreloaded(), runCycleGPP(), runCyclePPG(), leaveShootZone());
		} else if (obeliskID == 23) {
			autonAction = new SequentialAction(runPreloaded(), runCyclePPG(), runCycleGPP(), leaveShootZone());
		}
		Actions.runBlocking(new RaceAction(t -> {
			shooter.updateIntake(getRuntime());
			shooter.maintainVelocity(1350, true);
			return true;
		}, autonAction));
	}

	public static final double FAR_START_OFFSET = -15.0;

	private double getAdjustedX(double x) {
		return farStart ? x + FAR_START_OFFSET : x;
	}
	public Pose2d blueShootPose = new Pose2d(getAdjustedX(0), -4, Math.toRadians(-140));
	public Pose2d redShootPose = new Pose2d(getAdjustedX(-10), 17, Math.toRadians(130));

	public Action runPreloaded() {
        if (Common.alliance == Common.Alliance.Red) {
            return runPreloadedRed();
        } else {
            return runPreloadedBlue();
        }
    }

    public Action runCycleGPP() {
        if (Common.alliance == Common.Alliance.Red) {
            return runCycleGPPRed();
        } else {
            return runCycleGPPBlue();
        }
    }

//    public Action runCyclePGP() {
//        if (Common.alliance == Common.Alliance.Red) {
//            return runCyclePGPRed();
//        } else {
//            return runCyclePGPBlue();
//        }
//    }

    public Action runCyclePPG() {
        if (Common.alliance == Common.Alliance.Red) {
            return runCyclePPGRed();
        } else {
            return runCyclePPGBlue();
        }
    }
	public Action leaveShootZone() {
		if (Common.alliance == Common.Alliance.Red) {
			return leaveShootZoneRed();
		} else {
			return leaveShootZoneBlue();
		}
	}

	public Action leaveShootZoneRed() {
		return drive.actionBuilder(redShootPose)
				.setTangent(Math.toRadians(70))
				.splineToSplineHeading(new Pose2d(getAdjustedX(0), 55, Math.toRadians(90)), Math.toRadians(90))
				.build();
	}

	public Action leaveShootZoneBlue() {
		return drive.actionBuilder(blueShootPose)
				.setTangent(Math.toRadians(-70))
				.splineToSplineHeading(new Pose2d(getAdjustedX(12), -30, Math.toRadians(-90)), Math.toRadians(-90))
				.build();
	}
    private Action runPreloadedRed() {
        Pose2d start = drive.localizer.getPose();
        return drive.actionBuilder(start)
                // shoot preloaded
                .splineToSplineHeading(redShootPose, Math.toRadians(180))
                .stopAndAdd(preloadShootAction.get())
                .build();
    }

	private Action runCycleGPPRed() {
		return drive.actionBuilder(redShootPose)
				// first cycle
				.afterDisp(10, startIntakeAction.get())
				.setTangent(Math.toRadians(15))
				.splineToSplineHeading(new Pose2d(getAdjustedX(44), 60, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(20, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(redShootPose, Math.toRadians(225))
				.stopAndAdd(shootAction.get())
				.build();
	}

	private Action runCyclePGPRed() {
		return drive.actionBuilder(redShootPose)
				// second cycle
				.afterDisp(10, startIntakeAction.get())
				.setTangent(Math.toRadians(40))
				// Apply offset to X
				.splineToLinearHeading(new Pose2d(getAdjustedX(14), 56, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(10, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(redShootPose, Math.toRadians(230))
				.stopAndAdd(shootAction.get())
				.build();
	}

	private Action runCyclePPGRed() {
		return drive.actionBuilder(redShootPose)
				// third cycle
				.afterDisp(10, startIntakeAction.get())
				.setTangent(Math.toRadians(90))
				.splineToLinearHeading(new Pose2d(getAdjustedX(-4), 60, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(10, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(redShootPose, Math.toRadians(270))
				.stopAndAdd(shootAction.get())
				.build();
	}

	private Action runPreloadedBlue() {
		Pose2d start = drive.localizer.getPose();
		return drive.actionBuilder(start)
				// shoot preloaded
				.splineToSplineHeading(blueShootPose, Math.toRadians(-180))
				.stopAndAdd(preloadShootAction.get())
				.build();
	}

	private Action runCycleGPPBlue() {
		return drive.actionBuilder(blueShootPose)
				// first cycle
				.afterDisp(10, startIntakeAction.get())
				.setTangent(Math.toRadians(-20))
				.splineToSplineHeading(new Pose2d(getAdjustedX(61), -50, Math.toRadians(-90)), Math.toRadians(-90))
				.afterDisp(20, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(blueShootPose, Math.toRadians(-225))
				.stopAndAdd(shootAction.get())
				.build();
	}

	private Action runCyclePGPBlue() {
		return drive.actionBuilder(blueShootPose)
				// second cycle
				.afterDisp(10, startIntakeAction.get())
				.setTangent(Math.toRadians(-40))
				// Apply offset to X
				.splineToLinearHeading(new Pose2d(getAdjustedX(14), -56, Math.toRadians(-90)), Math.toRadians(-90))
				.afterDisp(10, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(blueShootPose, Math.toRadians(-230))
				.stopAndAdd(shootAction.get())
				.build();
	}

	private Action runCyclePPGBlue() {
		return drive.actionBuilder(blueShootPose)
				// third cycle
				.afterDisp(10, startIntakeAction.get())
				.setTangent(Math.toRadians(-90))
				// Apply offset to X
				.splineToLinearHeading(new Pose2d(getAdjustedX(8), -46, Math.toRadians(-90)), Math.toRadians(-90))
				.afterDisp(10, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(blueShootPose, Math.toRadians(-270))
				.stopAndAdd(shootAction.get())
				.build();
	}

//	public static Pose2d mapPose(Pose2d p) {
//		return new Pose2d(mapVector(p.position), mapRotation(p.heading));
//	}
//	public static Rotation2d mapRotation(Rotation2d rot) {
//		if (Common.alliance == Common.Alliance.Blue) {
//			return rot.inverse();
//		}
//		return rot;
//	}
//	public static double mapAngle(double r) {
//		return mapRotation(Rotation2d.exp(r)).log();
//	}
//	public static Vector2d mapVector(Vector2d position) {
//		if (Common.alliance == Common.Alliance.Blue) {
//			return new Vector2d(position.x+10, -position.y + 8);
//		}
//		return new Vector2d(position.x, position.y + 4);
//	}



	public void runInitLoop() {
		lastPoseInit = poseFilter.update(drive.localizer.getPose(), System.nanoTime());
		telemetry.addData("Obelisk ID", ll.getObeliskID(poseFilter.getCurrentPose()).map(x -> {
			obeliskID = x;
			switch (x) {
				case 21:
					return "GPP";
				case 22:
					return "PGP";
				case 23:
					return "PPG";
				default:
					return x.toString();
			}
		}).orElse("No obelisk apriltag visible"));

		ll.updateRobotOrientation(Math.toDegrees(lastPoseInit.heading.log()));
		Optional<Pair<Pose2d, Long>> pose = ll.localizeRobotMT1();
		if (pose.isPresent()) {
			Pose2d p = pose.get().first;
			TelemetryPacket packet = new TelemetryPacket();
			poseFilter.updateVision(p, pose.get().second);
			packet.fieldOverlay().setStroke("#4CAF50");
			Drawing.drawRobot(packet.fieldOverlay(), p);
			FtcDashboard.getInstance().sendTelemetryPacket(packet);
		} else {
			telemetry.addData("Pose", "AprilTags not available");
		}
		telemetry.addData("Alliance (press right bumper to change): ", Common.alliance.toString());
		if (Common.alliance == Common.Alliance.Blue && lastPoseInit.position.y > 0) {
			telemetry.addLine("Warning: Alliance set to Blue, but robot on Red side!");
		}
		if (Common.alliance == Common.Alliance.Red && lastPoseInit.position.y < 0) {
			telemetry.addLine("Warning: Alliance set to Red, but robot is on Blue side!");
		}
		if (gamepad1.rightBumperWasPressed()) {
			Common.alliance = Common.alliance.getOpposite();
		}
		telemetry.addData("Shot Offset (press left bumper to change): ", farStart ? "Far" : "Close");
		if (farStart && lastPoseInit.position.x > 40) {
			telemetry.addLine("Warning: Offset set to Far, but robot X position suggests Close start!");
		}
		if (!farStart && lastPoseInit.position.x < -40) {
			telemetry.addLine("Warning: Offset set to Close, but robot X position suggests Far start!");
		}

		if (gamepad1.leftBumperWasPressed()) {
			farStart = !farStart;
		}
		telemetry.update();
	}
}
