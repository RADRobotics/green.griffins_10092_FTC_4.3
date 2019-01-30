package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Rect;
import org.opencv.core.Size;

//dab
@Autonomous(name = "testAutoPath", group = "Prototyping")

public class testAutoPath extends LinearOpMode {
    Servo rightLock;// = hardwareMap.servo.get("rightLock");
    Servo leftLock;// = hardwareMap.servo.get("leftLock");
    DcMotor leftWheel1;// = hardwareMap.dcMotor.get("leftWheel");
    DcMotor leftWheel2;// = hardwareMap.dcMotor.get("leftWheel2");
    DcMotor rightWheel1;// = hardwareMap.dcMotor.get("rightWheel");
    DcMotor rightWheel2;// = hardwareMap.dcMotor.get("rightWheel2");
    DcMotor leftArm;// = hardwareMap.dcMotor.get("leftArm");
    DcMotor rightArm;// = hardwareMap.dcMotor.get("rightArm");
    DcMotor armExtendRight;// = hardwareMap.dcMotor.get("armExtend");
    DcMotor armExtendLeft;// = hardwareMap.dcMotor.get("armExtend2");

//    private GoldDetector detector;
//    double alignSize=70;
//    double alignX    = 270;//  (640 / 2) +0; // Center point in X Pixels
//    double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
//    double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
//    double scale = (640)/2;//-alignSize

    private ElapsedTime runtime = new ElapsedTime();
    private final double TICKS_PER_WHEEL_ROTATION = 1120;

    int subStep=0;
    private int step;

    private enum state {TURN_TO_WALL, MOVE_TO_WALL, MOVE_AWAY_FROM_WALL, TURN_TO_DEPOT, MOVE_TO_DEPOT}
    double stage = -4;

    int data[][]={{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160250,0,73,0},{0,0,160187,7,73,0},{0,0,160187,37,73,0},{0,0,160312,91,73,0},{0,0,160500,168,73,0},{0,0,160750,266,73,0},{0,-1,160875,372,73,0},{0,-2,160937,485,73,0},{0,-2,161125,586,73,0},{0,-3,161312,689,73,0},{-2,-3,161625,795,73,0},{-4,-4,162062,902,73,0},{-6,-4,162250,1008,72,0},{-6,-4,162250,1143,66,0},{-6,-4,162250,1311,64,0},{-6,-4,162250,1494,59,0},{-6,-4,162250,1666,55,0},{-6,-4,162250,1828,51,0},{-6,-4,162312,1997,45,0},{-6,-4,162312,2126,45,0},{-6,-5,162312,2245,40,0},{-6,-5,162312,2356,36,0},{-6,-5,162312,2470,32,0},{-6,-5,162312,2587,29,0},{-6,-5,162312,2711,27,0},{-6,-5,162312,2815,24,0},{-6,-5,162312,2919,21,0},{-6,-5,162312,3015,21,0},{-6,-5,162312,3113,17,0},{-6,-5,162312,3178,16,0},{-6,-5,162312,3232,15,0},{-6,-5,162312,3271,15,0},{-6,-5,162312,3296,13,0},{-6,-5,162312,3312,10,0},{-6,-5,162312,3320,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162250,3321,10,0},{-6,-5,162250,3321,10,0},{-6,-5,162250,3321,10,0},{-6,-5,162250,3321,10,0},{-6,-5,162250,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162375,3321,10,0},{-6,-5,162312,3321,10,0},{-6,-5,162187,3321,10,0},{-6,-5,162187,3321,10,0},{-6,-5,162250,3321,10,0},{-8,-5,162437,3321,10,0},{-13,-5,162750,3321,10,0},{-19,-3,163187,3321,10,0},{-26,-2,163562,3321,10,0},{-34,2,164187,3321,10,0},{-45,10,164687,3321,10,0},{-60,23,165062,3321,10,0},{-73,41,165375,3321,10,0},{-93,66,165250,3321,10,0},{-118,90,164625,3321,10,0},{-140,114,163937,3321,10,0},{-165,136,162875,3321,10,0},{-186,155,162375,3321,10,0},{-205,169,162500,3321,10,0},{-220,174,162937,3321,10,0},{-228,178,163437,3321,10,0},{-230,178,163562,3321,10,0},{-230,178,163500,3321,10,0},{-230,178,163437,3321,10,0},{-230,178,163437,3321,10,0},{-230,178,163437,3321,10,0},{-229,178,163375,3321,10,0},{-229,178,163375,3321,10,0},{-229,178,163375,3321,10,0},{-229,178,163375,3321,10,0},{-229,178,163375,3321,10,0},{-229,178,163375,3321,10,0},{-229,178,163375,3321,10,0},{-229,178,163375,3321,10,0},{-229,178,163375,3321,10,0},{-229,178,163375,3321,10,0},{-229,178,163375,3321,10,0},{-228,178,163375,3321,10,0},{-224,178,163187,3321,10,0},{-218,176,163000,3321,10,0},{-212,175,162875,3321,10,0},{-205,169,162875,3321,10,0},{-196,160,162875,3321,10,0},{-189,152,162875,3321,10,0},{-179,143,162875,3321,10,0},{-166,128,162875,3321,10,0},{-151,111,162937,3321,10,0},{-131,94,163062,3321,10,0},{-105,68,163187,3321,10,0},{-77,39,163062,3321,10,0},{-46,13,163000,3321,10,0},{-20,-9,163062,3321,10,0},{5,-34,163187,3321,10,0},{27,-56,163312,3321,10,0},{44,-73,163562,3321,10,0},{59,-93,164125,3321,10,0},{74,-113,164875,3321,10,0},{86,-132,165812,3321,10,0},{90,-137,165937,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165875,3321,10,0},{90,-137,165937,3321,10,0},{90,-137,165937,3321,10,0},{90,-137,166000,3321,10,0},{89,-137,166062,3321,10,0},{86,-134,166062,3321,10,0},{81,-129,166125,3321,10,0},{76,-125,166187,3321,10,0},{64,-124,166500,3321,10,0},{52,-120,166937,3321,10,0},{40,-113,167500,3321,10,0},{22,-106,167875,3321,10,0},{-2,-88,167625,3321,10,0},{-24,-67,166437,3321,10,0},{-47,-41,164562,3321,10,0},{-69,-13,162750,3321,10,0},{-100,14,161125,3321,10,0},{-136,35,160500,3321,10,0},{-170,48,160562,3321,10,0},{-202,62,161125,3321,9,0},{-232,74,161625,3321,9,0},{-250,83,161500,3321,9,0},{-254,87,161187,3321,8,0},{-254,87,161125,3321,5,0},{-254,87,161250,3321,5,0},{-254,87,161250,3321,5,0},{-254,87,161250,3321,5,0},{-254,87,161250,3321,5,0},{-254,87,161312,3321,5,0},{-254,87,161500,3321,5,0},{-254,87,161500,3321,5,0},{-254,87,161500,3321,5,0},{-254,87,161500,3321,5,0},{-254,87,161562,3321,5,0},{-254,87,161812,3321,5,0},{-254,87,163125,3321,5,0},{-258,81,164187,3321,5,0},{-264,72,165500,3321,5,0},{-270,66,167062,3321,5,0},{-278,55,169250,3321,5,0},{-288,44,171750,3321,5,0},{-297,35,173687,3321,5,0},{-308,27,176000,3321,5,0},{-317,19,178125,3321,5,0},{-325,10,-179937,3321,5,0},{-328,8,-179187,3321,5,0},{-328,8,-179250,3321,5,0},{-328,8,-179312,3321,5,0},{-328,8,-179375,3321,5,0},{-328,8,-179437,3321,5,0},{-328,8,-179500,3321,5,0},{-328,8,-179500,3321,5,0},{-328,8,-179500,3321,5,0},{-328,8,-179500,3321,5,0},{-328,8,-179500,3321,5,0},{-328,8,-179500,3321,5,0},{-328,8,-179625,3321,5,0},{-328,8,-179687,3321,5,0},{-328,8,-179875,3321,5,0},{-327,8,179687,3321,5,0},{-324,8,178687,3321,5,0},{-315,10,177312,3321,5,0},{-306,10,175937,3321,5,0},{-298,10,174562,3321,5,0},{-287,12,172687,3321,5,0},{-276,16,170875,3321,5,0},{-264,23,168625,3321,5,0},{-249,31,165937,3321,5,0},{-231,39,163062,3321,5,0},{-216,44,160500,3321,5,0},{-198,51,157375,3321,5,0},{-183,53,155312,3321,5,0},{-172,54,153937,3321,5,0},{-168,54,153375,3321,5,0},{-168,54,153312,3321,5,0},{-168,54,153375,3321,5,0},{-168,54,153375,3321,5,0},{-168,54,153437,3321,5,0},{-168,54,153437,3321,5,0},{-168,54,153437,3321,5,0},{-168,54,153562,3321,5,0},{-168,54,153562,3321,5,0},{-168,54,153562,3321,5,0},{-168,54,153562,3321,5,0},{-168,54,153750,3321,5,0},{-168,54,154000,3321,5,0},{-170,54,154625,3321,5,0},{-174,53,155812,3321,5,0},{-181,46,157375,3321,5,0},{-189,38,159375,3321,5,0},{-198,27,161750,3321,5,0},{-211,12,164750,3321,5,0},{-223,3,167937,3321,5,0},{-237,-11,171312,3321,5,0},{-254,-19,174375,3321,5,0},{-273,-32,178250,3321,5,0},{-297,-47,-177625,3321,5,0},{-315,-60,-174062,3321,5,0},{-336,-69,-170000,3321,5,0},{-357,-84,-166000,3321,5,0},{-376,-97,-162125,3321,5,0},{-392,-109,-159250,3321,5,0},{-400,-118,-157187,3321,5,0},{-401,-120,-156562,3321,5,0},{-401,-120,-156750,3321,5,0},{-401,-120,-156875,3321,5,0},{-401,-120,-156937,3321,5,0},{-401,-120,-156937,3321,5,0},{-401,-120,-156937,3321,5,0},{-401,-120,-156937,3321,5,0},{-401,-120,-156812,3321,5,0},{-401,-121,-156562,3321,5,0},{-401,-122,-156250,3321,5,0},{-403,-121,-155500,3321,5,0},{-407,-126,-154375,3321,5,0},{-414,-133,-152750,3321,5,0},{-424,-142,-150937,3321,5,0},{-437,-149,-148312,3321,5,0},{-450,-160,-145312,3321,5,0},{-465,-173,-142562,3321,5,0},{-483,-189,-138375,3321,5,0},{-505,-205,-133750,3321,5,0},{-529,-224,-128875,3321,5,0},{-549,-238,-124687,3321,5,0},{-570,-249,-119937,3321,5,0},{-590,-266,-116000,3321,5,0},{-608,-281,-112187,3321,5,0},{-627,-299,-107687,3321,5,0},{-648,-318,-102250,3321,5,0},{-665,-333,-97875,3321,5,0},{-680,-350,-94187,3321,5,0},{-693,-368,-89500,3321,5,0},{-701,-385,-86750,3321,5,0},{-702,-394,-84937,3321,5,0},{-702,-396,-84812,3321,5,0},{-702,-396,-85062,3321,5,0},{-702,-396,-85062,3321,5,0},{-702,-396,-85062,3321,5,0},{-702,-396,-85062,3321,5,0},{-702,-396,-85125,3321,5,0},{-702,-396,-85125,3321,5,0},{-702,-396,-85125,3321,5,0},{-702,-396,-85125,3321,5,0},{-702,-396,-85125,3321,5,0},{-702,-396,-85125,3321,5,0},{-702,-396,-85125,3321,5,0},{-702,-396,-85125,3321,5,0},{-702,-396,-85125,3321,5,0},{-702,-396,-85125,3321,5,0},{-702,-396,-85000,3321,5,0},{-702,-396,-85000,3321,5,0},{-702,-396,-84875,3321,5,0},{-702,-396,-84500,3321,5,0},{-702,-402,-83687,3321,5,0},{-702,-413,-82562,3321,5,0},{-702,-424,-81187,3321,5,0},{-702,-437,-79500,3321,5,0},{-702,-452,-77687,3321,5,0},{-702,-461,-75625,3321,5,0},{-703,-474,-73562,3321,5,0},{-704,-489,-71250,3321,5,0},{-712,-509,-67875,3321,5,0},{-721,-528,-64312,3321,5,0},{-736,-542,-60250,3321,5,0},{-755,-556,-56312,3321,5,0},{-774,-565,-53000,3321,5,0},{-794,-574,-49812,3321,5,0},{-816,-584,-46500,3321,5,0},{-835,-590,-43000,3321,5,0},{-857,-603,-38750,3321,5,0},{-878,-617,-35062,3321,5,0},{-895,-628,-31375,3321,5,0},{-909,-642,-27812,3321,5,0},{-921,-660,-23937,3321,5,0},{-928,-677,-20812,3321,5,0},{-936,-694,-18000,3321,5,0},{-941,-712,-14937,3321,5,0},{-944,-724,-12937,3321,5,0},{-944,-725,-12750,3321,5,0},{-944,-725,-13125,3321,5,0},{-944,-725,-13250,3321,5,0},{-944,-725,-13312,3321,5,0},{-944,-725,-13312,3321,5,0},{-944,-725,-13312,3321,5,0},{-944,-725,-13312,3321,5,0},{-944,-725,-13312,3321,5,0},{-944,-725,-13312,3321,5,0},{-944,-725,-13312,3321,5,0},{-944,-725,-13187,3321,5,0},{-944,-725,-12812,3321,5,0},{-950,-727,-11562,3321,5,0},{-958,-737,-9687,3321,5,0},{-971,-750,-7062,3321,5,0},{-986,-768,-3062,3321,5,0},{-1004,-789,1500,3321,5,0},{-1023,-814,5937,3321,5,0},{-1042,-841,11875,3321,5,0},{-1062,-865,17125,3321,5,0},{-1079,-895,22625,3321,5,0},{-1097,-929,28750,3321,5,0},{-1114,-958,34687,3321,5,0},{-1128,-987,39500,3321,5,0},{-1141,-1015,43937,3321,5,0},{-1154,-1041,49000,3321,5,0},{-1171,-1062,53687,3321,5,0},{-1188,-1083,57687,3321,5,0},{-1206,-1102,62312,3321,5,0},{-1222,-1119,66312,3321,5,0},{-1233,-1130,69000,3321,5,0},{-1234,-1133,69625,3321,5,0},{-1234,-1133,69062,3321,5,0},{-1234,-1133,69000,3321,5,0},{-1234,-1133,69000,3321,5,0},{-1234,-1133,69000,3321,5,0},{-1234,-1133,68937,3321,5,0},{-1234,-1133,68937,3321,5,0},{-1234,-1133,68937,3321,5,0},{-1234,-1133,68937,3321,5,0},{-1234,-1133,68937,3321,5,0},{-1234,-1133,68937,3321,5,0},{-1234,-1133,68937,3321,5,0},{-1234,-1133,69000,3321,5,0},{-1234,-1133,69250,3321,5,0},{-1237,-1134,69937,3321,5,0},{-1243,-1139,71125,3321,5,0},{-1250,-1147,72875,3321,5,0},{-1258,-1158,74875,3321,5,0},{-1272,-1175,78000,3321,5,0},{-1290,-1191,82625,3321,5,0},{-1308,-1212,86500,3321,5,0},{-1323,-1236,90562,3321,5,0},{-1337,-1260,95437,3321,5,0},{-1350,-1292,100312,3321,5,0},{-1362,-1321,105312,3321,5,0},{-1375,-1350,109625,3321,5,0},{-1388,-1378,114875,3321,5,0},{-1404,-1404,119250,3321,5,0},{-1420,-1430,124187,3321,5,0},{-1436,-1454,128375,3321,5,0},{-1453,-1478,133375,3321,5,0},{-1468,-1506,138437,3321,5,0},{-1482,-1529,143125,3321,5,0},{-1491,-1549,146812,3321,5,0},{-1500,-1565,149875,3321,5,0},{-1506,-1574,151812,3321,5,0},{-1509,-1575,152250,3321,5,0},{-1509,-1575,152187,3321,5,0},{-1509,-1575,152125,3321,5,0},{-1509,-1575,152062,3321,5,0},{-1509,-1575,152000,3321,5,0},{-1509,-1575,152000,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0},{-1509,-1575,151937,3321,5,0}};





            int pos = 0;

    int setL = 0;
    int setR = 0;

    double gyro = 0.0;
    int arm = 0;
    int extend = 0;
    int intake = 0;

    int errorR;
    int errorL;

    double p = .01;
    double i = .00001;
    double d = .00001;

    boolean aligned = false;

    public void runOpMode() throws InterruptedException {
        rightLock = hardwareMap.servo.get("rightLock");
        leftLock = hardwareMap.servo.get("leftLock");
        leftWheel1 = hardwareMap.dcMotor.get("leftWheel");
        leftWheel2 = hardwareMap.dcMotor.get("leftWheel2");
        rightWheel1 = hardwareMap.dcMotor.get("rightWheel");
        rightWheel2 = hardwareMap.dcMotor.get("rightWheel2");
        leftArm = hardwareMap.dcMotor.get("leftArm");
        rightArm = hardwareMap.dcMotor.get("rightArm");
        armExtendRight = hardwareMap.dcMotor.get("armExtend");
        armExtendLeft = hardwareMap.dcMotor.get("armExtend2");

        //rightWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        step = -1;

//        telemetry.addData("Status", "DogeCV 2018.0 - Gold Detector test");
//        // Setup detector
//        detector = new GoldDetector(); // Create detector
//        detector.setAdjustedSize(new Size(480, 270)); // Set detector size
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
//        detector.useDefaults(); // Set default detector settings
//        // Optional tuning
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005;
//
//        detector.ratioScorer.weight = 5;
//        detector.ratioScorer.perfectRatio = 1.0;
//        detector.enable(); // Start detector


        //prepares and resets robot by resetting encoders
        //resetDriveEncoders();

        //prepares robot for arm movement by setting arm motors to go to a certain position-modifiable by user input (the d-pad)
//        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //HSV value array

        //Sends color sensor input values to the phone
        waitForStart();


        rightWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            telemetry.update();


           // telemetry.addLine()
               //     .addData("step", step);
            telemetry.addLine()
                    .addData("rightWheel1: ", leftWheel1.getCurrentPosition())
                    .addData("rightWheel1: ", rightWheel1.getCurrentPosition());
            //.addData("TICKS_PER_WHEEL_ROTATION: ", TICKS_PER_WHEEL_ROTATION);
          //  telemetry.addData("arm::::::", leftArm.getCurrentPosition());
            //telemetry.addData("armExtend", armExtendLeft.getCurrentPosition());
            //telemetry.addData("armExtend2", armExtendRight.getCurrentPosition());

            if (step == -1) {
                if (runtime.seconds() > .025) {
                    setL = data[pos][0];
                    setR = data[pos][1];
                    gyro = ((double)data[pos][2])/1000;
                    arm = data[pos][3];
                    extend = data[pos][4];
                    intake= data[pos][5];
                    pos++;
                    runtime.reset();
                }
                errorR= setR-rightWheel1.getCurrentPosition();
                errorL = setL-leftWheel1.getCurrentPosition();
                double pr;
                double pl;

                pr= errorR*p;
                pl=errorL*p;

                rightWheel1.setPower(pr);
                rightWheel2.setPower(pr);
                leftWheel1.setPower(pl);
                leftWheel2.setPower(pl);

                telemetry.addData("pr",pr);
                telemetry.addData("pl",pl);
                telemetry.addData("errorL:", errorL);
                telemetry.addData("errorR",errorR);



            }
//            if (step == 0) {
//                leftLock.setPosition(.25);
//                rightLock.setPosition(.4);
//                if(leftArm.getCurrentPosition()>-4250) {
//                    //leftArm.setPower(.3);
//                    rightArm.setPower(-.3);
//
//                }
//                else if (leftArm.getCurrentPosition()>-5000){
//                    //leftArm.setPower(.65);
//                    rightArm.setPower(-.65);
//                    if(rightWheel1.getCurrentPosition()>-400) {
//                        leftWheel2.setPower(-.4);
//                        leftWheel1.setPower(-.4);
//                        rightWheel2.setPower(-.4);
//                        rightWheel1.setPower(-.4);
//                    }
//                    else{
//                        leftWheel2.setPower(0);
//                        leftWheel1.setPower(0);
//                        rightWheel2.setPower(0);
//                        rightWheel1.setPower(0);
//                    }
//                }
//                else{
//                    leftWheel2.setPower(0);
//                    leftWheel1.setPower(0);
//                    rightWheel2.setPower(0);
//                    rightWheel1.setPower(0);
//                   // leftArm.setPower(0);
//                    rightArm.setPower(0);
//                    step=1;
//                    runtime.reset();
//                }
//
//            }
//            if(step==1){
//                //leftArm.setPower(-.55);
//                 rightArm.setPower(.55);
//                if(runtime.seconds()>2.5){
//                    leftArm.setPower(0);
//                    rightArm.setPower(0);
//                    runtime.reset();
//                    step=2;
//                    rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    rightWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    leftWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
//            }
//            if(step==2) {
//                Rect bestRect = detector.getFoundRect();
//
//                double xPos = bestRect.x + (bestRect.width / 2);
//
//                if (xPos < alignXMax && xPos > alignXMin) {
//                    aligned = true;
//                } else {
//                    aligned = false;
//                }
//                telemetry.addData("aligned ", aligned);
//
//                telemetry.addData("xpos ", xPos);
//                telemetry.addData("amax ", alignXMax);
//                telemetry.addData("amin ", alignXMin);
//                if(!(xPos>0)){
//                    if(runtime.seconds()<4) {
//                        rightWheel1.setPower(0);
//                        rightWheel2.setPower(0);
//                        leftWheel1.setPower(0);
//                        leftWheel2.setPower(0);
//                    }
//                    telemetry.addLine("not detected");
//
//                    if(runtime.seconds()>4 && runtime.seconds()<5){
//                        if(rightWheel1.getCurrentPosition()>-150) {
//                            rightWheel1.setPower(-.4);
//                            rightWheel2.setPower(-.4);
//                            leftWheel1.setPower(.4);
//                            leftWheel2.setPower(.4);
//                        }
//                        else{
//                            rightWheel1.setPower(0);
//                            rightWheel2.setPower(0);
//                            leftWheel1.setPower(0);
//                            leftWheel2.setPower(0);
//                        }
//                    }
//                    if(runtime.seconds()>5 && runtime.seconds()<7){
//                        if(rightWheel1.getCurrentPosition()<150) {
//                            rightWheel1.setPower(.4);
//                            rightWheel2.setPower(.4);
//                            leftWheel1.setPower(-.4);
//                            leftWheel2.setPower(-.4);
//                        }
//                        else{
//                            rightWheel1.setPower(0);
//                            rightWheel2.setPower(0);
//                            leftWheel1.setPower(0);
//                            leftWheel2.setPower(0);
//                        }
//                    }
//                    if(runtime.seconds()>7){
//                        if(rightWheel1.getCurrentPosition()>1) {
//                            rightWheel1.setPower(-.4);
//                            rightWheel2.setPower(-.4);
//                            leftWheel1.setPower(.4);
//                            leftWheel2.setPower(.4);
//                        }
//                        else{
//                            runtime.reset();
//                            step++;
//                            rightWheel1.setPower(0);
//                            rightWheel2.setPower(0);
//                            leftWheel1.setPower(0);
//                            leftWheel2.setPower(0);
//                        }
//
//                    }
//                }
//                else if (xPos > alignXMax) {
//                    double power = ((xPos - alignXMax) / scale) * .12 + .11;
//                    rightWheel1.setPower(-power);
//                    rightWheel2.setPower(-power);
//                    leftWheel1.setPower(power);
//                    leftWheel2.setPower(power);
//                    telemetry.addData("powL: ", power);
//                    telemetry.addLine("turning left");
//                    runtime.reset();
//                } else if (xPos < alignXMin) {
//                    double power = ((alignXMin - xPos) / scale) * .12 + .11 ;
//                    rightWheel1.setPower(power);
//                    rightWheel2.setPower(power);
//                    leftWheel1.setPower(-power);
//                    leftWheel2.setPower(-power);
//                    telemetry.addData("powR: ", power);
//                    telemetry.addLine("turning right");
//                    runtime.reset();
//                } else {
//                    rightWheel1.setPower(0);
//                    rightWheel2.setPower(0);
//                    leftWheel1.setPower(0);
//                    leftWheel2.setPower(0);
//                    telemetry.addLine("found");
//                    telemetry.addData("secks: ", runtime.seconds());
//                    if(runtime.seconds()>.3){
//                        runtime.reset();
//                        step++;
//                        //resetDriveEncoders();
//                    }
//                }
//
//
//            }
//            else if (step==3){
//                if(rightWheel1.getCurrentPosition()<1500) {
//                    leftWheel2.setPower(.6);
//                    leftWheel1.setPower(.6);
//                    rightWheel2.setPower(.6);
//                    rightWheel1.setPower(.6);
//                }
//                else {
//                    rightWheel1.setPower(0);
//                    rightWheel2.setPower(0);
//                    leftWheel1.setPower(0);
//                    leftWheel2.setPower(0);
//                }
//                //stage = 2;
//            }
//
//    /*
////sets the step of where we are in the process to one
//            // step = 1;
////enables the if statement for lander_unlatched
//            lander_unlatched = true;
////moves the robot from the lander to where the samples are
//            if (step == 1) {
////                leftWheel.setPower(0.7);
////                rightWheel.setPower(0.5);
////                leftWheel.setTargetPosition(1621);
////                rightWheel.setTargetPosition(820);
//                step = 2;
//            }
//            if (step == 2) {
////                if (hsvValues[0] > 50) {
////                    leftWheel.setPower(-0.7);
////                    rightWheel.setPower(-0.5);
////                    leftWheel.setTargetPosition(1221);
//                step = 3;
//            }
////            if step 2 is finished, makes the robot turn toward the wall, then drives against the wall.
//            if (step == 3) {
//                moveToDepot();
//            }
//            //1150 Target Units == about 1 foot, 96 Units/inch
////sets the wheel power to 0 to limit movement
////                if (rightWheel.getCurrentPosition() == -rotation) {
//            // rightWheel.setPower(0);
//            //leftWheel.setPower(0);
////                }
////               if (leftWheel.getCurrentPosition() == 100) {
////                    leftWheel.setPower(0);
////                    rightWheel.setPower(0);
////                }
//
//            sample_ready = true;
//
////        if (sample_ready) {
////
//        }
//
//        */
//        }
//    }
//
//    private void resetDriveEncoders() {
//        telemetry.addLine("RESET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" +
//                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//        leftWheel1.setPower(0);
//        leftWheel2.setPower(0);
//        rightWheel1.setPower(0);
//        rightWheel2.setPower(0);
//        leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftWheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightWheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//    }
//
//    private void moveToDepot() throws InterruptedException {
//
//        if ((subStep * 2) % 2 == 1) {
//            resetDriveEncoders();
//            subStep += 0.5;
//        } else if (subStep == state.TURN_TO_WALL.ordinal()) {
//            // turns the robot 90 degrees counter clockwise
//            double target = .75;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        }
////
//        else if (subStep == state.MOVE_TO_WALL.ordinal()) {
//            // moves the robot forward up against the wall
//
//            double target = 3;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        } else if (subStep == state.MOVE_AWAY_FROM_WALL.ordinal()) {
//            // moves the robot backward away from the wall
//            double target = -1;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !leftWheel2.isBusy()) {
//                subStep += 0.5;
//
//            }
//        } else if (subStep == state.TURN_TO_DEPOT.ordinal()) {
//            // turns the robot counter-clockwise to line up with the depot
//            double target = 1;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        } else if (subStep == state.MOVE_TO_DEPOT.ordinal()) {
//            // moves the robot forward up to the depot
//            double target = 5;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        }
            //step = 4;
        }
    }
}
