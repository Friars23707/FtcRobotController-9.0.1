/*.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(-90)))
                                .strafeLeft(20)
                                .strafeTo(new Vector2d(40, -32))
                                .turn(Math.toRadians(-90))
                                .forward(10)
                                .back(10)
                                .strafeTo(new Vector2d(42, -40))
                                .strafeRight(31)
                                .back(16)
                                .build()
                ); RED RIGHT CLOSE */



/*.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(-90)))
                                .strafeLeft(20)
                                .strafeTo(new Vector2d(40, -23))
                                .turn(Math.toRadians(-90))
                                .forward(20)
                                .back(20)
                                .strafeTo(new Vector2d(42, -34))
                                .strafeRight(25)
                                .back(16)
                                .build()
                ); RED CENTER CLOSE */



/*.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-47, -50))
                                .turn(Math.toRadians(-90))
                                .strafeRight(30)
                                .strafeLeft(10)
                                .back(8)
                                .waitSeconds(0.2)
                                .back(7)
                                .strafeRight(25)
                                .back(75)
                                .strafeTo(new Vector2d(42, -29))
                                .strafeRight(20)
                                .back(16)
                                .build()
                ); RED LEFT FAR */



/*.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                                .back(65)
                                .forward(22)
                                .waitSeconds(0.2)
                                .back(12)
                                .turn(Math.toRadians(-90))
                                .back(75)
                                .strafeTo(new Vector2d(42, -34))
                                .strafeRight(25)
                                .back(16)
                                .build()
                ); RED CENTER FAR */



/*.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-50, -30))
                                .turn(Math.toRadians(90))
                                .forward(18)
                                .waitSeconds(0.2)
                                .back(15)
                                .strafeLeft(20)
                                .turn(Math.toRadians(180))
                                .back(65)
                                .strafeTo(new Vector2d(42, -40))
                                .strafeRight(31)
                                .back(16)
                                .build()
                ); RED RIGHT FAR */