import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

// import com.igknighters.commands.autos.Autos;
// import com.igknighters.constants.ConstValues;
// import com.igknighters.constants.RobotSetup;
// import com.igknighters.constants.RobotSetup.RobotID;
// import com.igknighters.util.RobotExtension.Robo;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class RobotTest {

    @BeforeEach
    protected void startSim() {
        GlobalState.setUnitTest(true);
        assert HAL.initialize(500, 0);
    }

    @AfterEach
    protected void teardownSim() {
        HAL.exitMain();
        HAL.shutdown();
        GlobalState.restoreDefaultState();
        var cmdScheduler = CommandScheduler.getInstance();
        cmdScheduler.cancelAll();
        cmdScheduler.getActiveButtonLoop().clear();
        cmdScheduler.getDefaultButtonLoop().clear();
        cmdScheduler.clearComposedCommands();
        cmdScheduler.unregisterAllSubsystems();
    }

    @Test
    public void testRobotSetup() {
        for (RobotID id : RobotID.values()) {
            if (id == RobotID.Unlabeled) {
                continue;
            }

            RobotSetup.testOverrideRobotID(id);

            com.igknighters.ConstantHelper.applyRoboConst(ConstValues.class);

            new RobotContainer();

            CommandScheduler.getInstance().getActiveButtonLoop().clear();
            CommandScheduler.getInstance().getDefaultButtonLoop().clear();

            System.gc();
        }
    }

    @Test
    public void testAuto(@Robo Robot robot) {
        RobotSetup.testOverrideRobotID(RobotID.SIM_CRASH);
        Pose2d desiredEndPose = new Pose2d(
                new Translation2d(3.0, 7.0),
                new Rotation2d());

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

        // meters
        final double translationTolerance = 0.2;

        Autos.setAutoOverrideTest(new ProxyCommand(() -> new PathPlannerAuto("1 Meter Auto")));
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);

        robot.withAutonomousPeriodicTest(robo -> {
            boolean isFinished = GlobalState.getLocalizedPose()
                    .getTranslation()
                    .getDistance(desiredEndPose.getTranslation()) < translationTolerance;

            if (isFinished) {
                robo.finishUnitTestRobot();
            } else if (robo.getElapsedTime() > 2.5) {
                throw new RuntimeException(
                        "Auto took to long, ended at "
                                + GlobalState.getLocalizedPose().toString());
            }
        });

        robot.runTest(3);
    }
}