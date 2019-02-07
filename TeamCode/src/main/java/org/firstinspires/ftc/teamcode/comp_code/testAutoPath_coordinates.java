package org.firstinspires.ftc.teamcode.comp_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.hmap;

import java.io.File;

import static java.lang.Integer.parseInt;

//dab
@Autonomous(name = "testAutoPath_coordinates", group = "Prototyping")

public class testAutoPath_coordinates extends LinearOpMode {
    hmap hwmap = new hmap();

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



    //private ArrayList<int[]> dataLeft = new ArrayList<int[]>();
    int data[][];//null;// {{0,0,173187,0,0,0,0},{0,0,173187,0,0,0,0},{0,0,173187,0,0,0,0},{0,0,173312,0,0,0,0},{0,0,173562,0,0,0,0},{3,2,173812,0,0,0,0},{5,4,173875,0,0,0,0},{9,8,173937,0,0,0,0},{17,15,174125,0,0,0,0},{29,24,174250,0,0,0,0},{41,34,174437,0,0,0,0},{55,44,174812,0,0,0,0},{71,53,175312,0,0,0,0},{86,62,175875,0,0,0,0},{96,69,176375,0,0,0,0},{108,75,176875,0,0,0,0},{120,80,177562,0,0,0,0},{131,85,178000,0,0,0,0},{145,92,178875,0,0,0,0},{161,99,179687,0,0,0,0},{179,106,-179125,0,0,0,0},{200,115,-177875,0,0,0,0},{221,125,-176625,0,0,0,0},{250,137,-175062,0,0,0,0},{275,153,-173875,0,0,0,0},{307,172,-172812,0,0,0,0},{335,189,-171562,0,0,0,0},{365,204,-170250,0,0,0,0},{395,217,-168500,0,0,0,0},{426,228,-166687,0,0,0,0},{453,237,-164375,0,0,0,0},{487,247,-162500,0,0,0,0},{520,256,-160250,0,0,0,0},{554,266,-157812,0,0,0,0},{586,273,-155250,0,0,0,0},{625,286,-152687,0,0,0,0},{658,296,-150250,0,0,0,0},{697,308,-147625,0,0,0,0},{735,321,-145312,0,0,0,0},{761,329,-143687,0,0,0,0},{802,346,-141312,0,0,0,0},{837,364,-139562,0,0,0,0},{868,383,-138375,0,0,0,0},{901,405,-137250,0,0,0,0},{934,424,-135937,0,0,0,0},{971,444,-134062,0,0,0,0},{1007,461,-132437,0,0,0,0},{1044,476,-130500,0,0,0,0},{1072,491,-128687,0,0,0,0},{1098,507,-127875,0,0,0,0},{1124,525,-127125,0,0,0,0},{1154,545,-126125,0,0,0,0},{1190,566,-124750,0,0,0,0},{1230,591,-123125,0,0,0,0},{1272,619,-121687,0,0,0,0},{1307,641,-120625,0,0,0,0},{1344,670,-119750,0,0,0,0},{1375,693,-119000,0,0,0,0},{1403,710,-117812,0,0,0,0},{1432,723,-116437,0,0,0,0},{1458,736,-115062,0,0,0,0},{1483,745,-113562,0,0,0,0},{1514,762,-112187,0,0,0,0},{1552,786,-110625,0,0,0,0},{1593,814,-109562,0,0,0,0},{1633,844,-108500,0,0,0,0},{1683,881,-106937,0,0,0,0},{1725,911,-105937,0,0,0,0},{1776,947,-104500,0,0,0,0},{1823,979,-103125,0,0,0,0},{1863,1007,-101875,0,0,0,0},{1902,1035,-100750,0,0,0,0},{1938,1060,-99687,0,0,0,0},{1982,1094,-98375,0,0,0,0},{2021,1120,-97437,0,0,0,0},{2054,1143,-96562,0,0,0,0},{2082,1167,-95750,0,0,0,0},{2121,1197,-94937,0,0,0,0},{2159,1227,-94375,0,0,0,0},{2188,1249,-93750,0,0,0,0},{2216,1272,-92937,0,0,0,0},{2238,1288,-92562,0,0,0,0},{2262,1307,-92000,0,0,0,0},{2300,1335,-90937,0,0,0,0},{2334,1359,-90125,0,0,0,0},{2369,1387,-89250,0,0,0,0},{2407,1419,-88562,0,0,0,0},{2441,1445,-87937,0,0,0,0},{2486,1479,-86937,0,0,0,0},{2518,1503,-86125,0,0,0,0},{2559,1534,-85125,0,0,0,0},{2600,1566,-84250,0,0,0,0},{2644,1600,-83250,0,0,0,0},{2682,1632,-82500,0,0,0,0},{2718,1662,-81812,0,0,0,0},{2765,1700,-81125,0,0,0,0},{2801,1731,-80687,0,0,0,0},{2833,1759,-80125,0,0,0,0},{2864,1785,-79750,0,0,0,0},{2900,1817,-79187,0,0,0,0},{2939,1848,-78625,0,0,0,0},{2984,1887,-77937,0,0,0,0},{3011,1908,-77437,0,0,0,0},{3049,1940,-76625,0,0,0,0},{3096,1977,-75812,0,0,0,0},{3135,2007,-75000,0,0,0,0},{3183,2044,-73687,0,0,0,0},{3237,2084,-72625,0,0,0,0},{3278,2117,-71687,0,0,0,0},{3327,2156,-70750,0,0,0,0},{3372,2193,-69937,0,0,0,0},{3416,2229,-69125,0,0,0,0},{3452,2257,-68187,0,0,0,0},{3500,2293,-67125,0,0,0,0},{3549,2330,-66062,0,0,0,0},{3578,2351,-65500,0,0,0,0},{3604,2373,-64937,0,0,0,0},{3638,2400,-64125,0,0,0,0},{3676,2429,-63312,0,0,0,0},{3702,2451,-62812,0,0,0,0},{3731,2474,-62250,0,0,0,0},{3763,2498,-61500,0,0,0,0},{3794,2520,-60687,0,0,0,0},{3822,2543,-60125,0,0,0,0},{3850,2567,-59625,0,0,0,0},{3887,2599,-59125,0,0,0,0},{3920,2627,-58625,0,0,0,0},{3947,2651,-58187,0,0,0,0},{3976,2675,-57750,0,0,0,0},{4011,2705,-57312,0,0,0,0},{4046,2739,-57187,0,0,0,0},{4073,2763,-57000,0,0,0,0},{4104,2791,-56812,0,0,0,0},{4129,2815,-56687,0,0,0,0},{4148,2832,-56687,0,0,0,0},{4167,2851,-56562,0,0,0,0},{4194,2879,-56500,0,0,0,0},{4223,2908,-56500,0,0,0,0},{4248,2934,-56625,0,0,0,0},{4271,2957,-56687,0,0,0,0},{4294,2981,-56750,0,0,0,0},{4315,3002,-56812,0,0,0,0},{4339,3026,-56875,0,0,0,0},{4363,3050,-56937,0,0,0,0},{4393,3080,-56937,0,0,0,0},{4413,3100,-56937,0,0,0,0},{4437,3125,-56937,0,0,0,0},{4462,3150,-57000,0,0,0,0},{4486,3175,-57062,0,0,0,0},{4511,3200,-57125,0,0,0,0},{4535,3226,-57187,0,0,0,0},{4561,3250,-57250,0,0,0,0},{4575,3265,-57312,0,0,0,0},{4591,3280,-57312,0,0,0,0},{4606,3295,-57375,0,0,0,0},{4618,3307,-57375,0,0,0,0},{4629,3319,-57375,0,0,0,0},{4637,3325,-57375,0,0,0,0},{4643,3332,-57312,0,0,0,0},{4650,3338,-57375,0,0,0,0},{4654,3343,-57375,0,0,0,0},{4658,3347,-57375,0,0,0,0},{4660,3349,-57375,0,0,0,0},{4662,3351,-57375,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0},{4662,3351,-57437,0,0,0,0}};

    //{{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{-1,0,160062,0,0,0},{-2,0,160062,0,0,0},{-3,0,160062,0,0,0},{-5,0,160062,0,0,0},{-5,2,160062,0,0,0},{-7,4,160062,0,0,0},{-11,8,160000,0,0,0},{-18,14,160000,0,0,0},{-25,22,160000,0,0,0},{-37,36,160000,0,0,0},{-54,52,159937,0,0,0},{-70,68,159937,0,0,0},{-89,89,159812,0,0,0},{-107,110,159562,0,0,0},{-124,129,159437,0,0,0},{-147,155,159062,0,0,0},{-172,187,158562,0,0,0},{-199,219,158062,0,0,0},{-225,251,157437,0,0,0},{-256,288,156812,0,0,0},{-285,326,156125,0,0,0},{-318,374,154812,0,0,0},{-345,418,153312,0,0,0},{-378,470,151250,0,0,0},{-405,511,149875,0,0,0},{-427,549,148250,0,0,0},{-454,596,146375,0,0,0},{-479,646,144125,0,0,0},{-501,698,141187,0,0,0},{-522,745,138500,0,0,0},{-544,797,135625,0,0,0},{-563,846,132937,0,0,0},{-577,889,130062,0,0,0},{-592,935,126687,0,0,0},{-606,980,123437,0,0,0},{-623,1022,120937,0,0,0},{-643,1064,118812,0,0,0},{-662,1101,116687,0,0,0},{-686,1143,114875,0,0,0},{-709,1180,113437,0,0,0},{-736,1223,112062,0,0,0},{-766,1266,110750,0,0,0},{-804,1313,109625,0,0,0},{-845,1357,109062,0,0,0},{-889,1399,109062,0,0,0},{-938,1442,109437,0,0,0},{-985,1479,110000,0,0,0},{-1033,1517,110625,0,0,0},{-1077,1550,111875,0,0,0},{-1123,1579,113437,0,0,0},{-1166,1600,115062,0,0,0},{-1215,1621,117750,0,0,0},{-1258,1635,120750,0,0,0},{-1299,1650,123312,0,0,0},{-1338,1667,125750,0,0,0},{-1383,1695,127750,0,0,0},{-1437,1734,129062,0,0,0},{-1489,1772,130312,0,0,0},{-1540,1809,131937,0,0,0},{-1592,1841,133625,0,0,0},{-1646,1875,135812,0,0,0},{-1715,1916,138687,0,0,0},{-1776,1954,140812,0,0,0},{-1827,1986,142750,0,0,0},{-1875,2014,144562,0,0,0},{-1928,2042,146812,0,0,0},{-1972,2067,148812,0,0,0},{-2015,2090,151000,0,0,0},{-2058,2113,153000,0,0,0},{-2101,2137,154812,0,0,0},{-2132,2154,156250,0,0,0},{-2168,2178,157625,0,0,0},{-2198,2197,158562,0,0,0},{-2225,2218,159312,0,0,0},{-2247,2238,159687,0,0,0},{-2270,2255,160062,0,0,0},{-2286,2270,160187,0,0,0},{-2301,2283,160250,0,0,0},{-2308,2290,160250,0,0,0},{-2311,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2292,160250,0,0,0},{-2308,2292,160250,0,0,0},{-2304,2288,160187,0,0,0},{-2299,2283,160187,0,0,0},{-2293,2275,160187,0,0,0},{-2282,2266,160187,0,0,0},{-2269,2253,160187,0,0,0},{-2253,2238,160125,0,0,0},{-2236,2221,160000,0,0,0},{-2217,2203,159875,0,0,0},{-2194,2182,159687,0,0,0},{-2171,2163,159437,0,0,0},{-2143,2137,159062,0,0,0},{-2107,2107,158625,0,0,0},{-2077,2081,158187,0,0,0},{-2039,2051,157500,0,0,0},{-2006,2027,156625,0,0,0},{-1969,2003,155312,0,0,0},{-1932,1980,154062,0,0,0},{-1892,1956,152375,0,0,0},{-1851,1932,150187,0,0,0},{-1802,1906,148062,0,0,0},{-1752,1883,145687,0,0,0},{-1703,1859,143125,0,0,0},{-1651,1835,140375,0,0,0},{-1602,1814,137312,0,0,0},{-1549,1795,134000,0,0,0},{-1498,1779,130562,0,0,0},{-1448,1764,126937,0,0,0},{-1401,1749,123312,0,0,0},{-1348,1734,120375,0,0,0},{-1307,1723,117500,0,0,0},{-1259,1706,114250,0,0,0},{-1218,1691,111437,0,0,0},{-1176,1671,109125,0,0,0},{-1142,1652,107750,0,0,0},{-1107,1630,106312,0,0,0},{-1075,1607,105437,0,0,0},{-1041,1577,104937,0,0,0},{-1009,1546,104812,0,0,0},{-977,1513,104937,0,0,0},{-942,1475,105187,0,0,0},{-907,1434,105750,0,0,0},{-871,1392,106500,0,0,0},{-834,1343,107687,0,0,0},{-804,1298,109000,0,0,0},{-775,1252,110687,0,0,0},{-747,1208,112375,0,0,0},{-716,1157,114500,0,0,0},{-679,1094,116937,0,0,0},{-648,1051,118750,0,0,0},{-617,997,121000,0,0,0},{-590,950,123125,0,0,0},{-563,894,125937,0,0,0},{-536,845,128125,0,0,0},{-515,802,130687,0,0,0},{-492,750,133375,0,0,0},{-475,700,136875,0,0,0},{-462,659,139500,0,0,0},{-445,615,142000,0,0,0},{-430,577,144687,0,0,0},{-407,524,147375,0,0,0},{-386,489,149187,0,0,0},{-360,452,150437,0,0,0},{-332,420,151000,0,0,0},{-300,385,151250,0,0,0},{-266,351,151312,0,0,0},{-236,317,151562,0,0,0},{-208,285,152000,0,0,0},{-183,255,152312,0,0,0},{-159,226,152812,0,0,0},{-136,200,153312,0,0,0},{-118,177,153875,0,0,0},{-98,148,154812,0,0,0},{-84,127,155562,0,0,0},{-68,103,156250,0,0,0},{-54,82,157062,0,0,0},{-40,62,157812,0,0,0},{-29,43,158562,0,0,0},{-17,26,159187,0,0,0},{-9,13,159625,0,0,0},{-1,0,160125,0,0,0},{4,-6,160375,0,0,0},{9,-11,160437,0,0,0},{14,-15,160437,0,0,0},{15,-17,160437,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0}};





    int pos = 0;

    int setL = 0;
    int setR = 0;

    int speedL = 0;
    int speedR = 0;

    double gyro = 0.0;
    int arm = 0;
    int extend = 0;
    int intake = 0;
    int intake2 = 0;

    int errorR;
    int errorL;

    int sumErrorR;
    int sumErrorL;

    int previousErrorR;
    int previousErrorL;

    int previousLeftPos;
    int previousRightPos;

    double sumX =0;
    double sumY = 0;
    double sumAngle = 0;

    double sumTargetX = 0;
    double sumTargetY = 0;
    double sumTargetAngle = 0;

    //double Kp = .002;
    //double Ki = 0.0001;
    //double Kd = 0.001;

    double Kp = 0.0015;
    double Ki = 0.00015;
    double Kd = 0.001;
    //double Kg = 0.001;
    double Kg = 0;

    double Kf = 0.007;

    boolean aligned = false;

    String csvData = "leftTargetPos,leftCurrentPos,rightTargetPos,rightCurrentPos,P,I,D,F,G\r\n";

    public void runOpMode() throws InterruptedException {

        telemetry.addData("reading left...","");
        String readfile = "left.csv";
        File fileR = AppUtil.getInstance().getSettingsFile(readfile);
        String sdata = ReadWriteFile.readFile(fileR);
        String[] split1 = sdata.split("@");
        int[][] dataLeft = new int[split1.length][7];
        for (int i=0;i<split1.length-1;i++){
            String[] split2 = split1[i].split(",");
            for(int p=0;p<split2.length;p++){
                dataLeft[i][p]=parseInt(split2[p]);
            }
        }

        telemetry.addData("reading center...","");
        File fileR2 = AppUtil.getInstance().getSettingsFile("center.csv");
        String sdata2 = ReadWriteFile.readFile(fileR2);
        String[] split1c = sdata2.split("@");
        int[][] dataCenter = new int[split1c.length][7];
        for (int i=0;i<split1c.length-1;i++){
            String[] split2 = split1c[i].split(",");
            for(int p=0;p<split2.length;p++){
                dataCenter[i][p]=parseInt(split2[p]);
            }

        }

        telemetry.addData("reading right...","");
        File fileRr = AppUtil.getInstance().getSettingsFile("right.csv");
        String sdatar = ReadWriteFile.readFile(fileRr);
        String[] split1r = sdatar.split("@");
        int[][] dataRight = new int[split1r.length][7];
        for (int i=0;i<split1r.length-1;i++){
            String[] split2 = split1r[i].split(",");
            for(int p=0;p<split2.length;p++){
                dataRight[i][p]=parseInt(split2[p]);
            }

        }

        telemetry.addData("done reading","");
            //System.out.println(a);
        //for loop to create data array from string to do

        hwmap.init(hardwareMap);
        hwmap.initGyro();
        hwmap.reset();


        step = -2;

        waitForStart();
        hwmap.reset();

        while (opModeIsActive()) {
            telemetry.update();


           // telemetry.addLine()
               //     .addData("step", step);
            telemetry.addLine()
                    .addData("", hwmap.print());
            //telemetry.addData("gyro",hwmap.gyro.getHeading());
if(step==-2){

    telemetry.addData("load left:X, load center: A, load right: B","");
    if(gamepad1.x || gamepad2.x){
        data = dataLeft;
        step=-1;
    }
    if(gamepad1.a || gamepad2.a){
        data = dataCenter;
        step=-1;
    }
    if(gamepad1.b || gamepad2.b){
        data = dataRight;
        step=-1;
    }
}

            if (step == -1) {
                if (runtime.seconds() > .025) {
                    setL = data[pos][1];
                    setR = data[pos][0];
                    speedL =  data[pos+1][1] - data[pos][1];
                    speedR =  data[pos+1][0] - data[pos][0];
                    gyro = ((double)data[pos][2])/1000;
                    arm = data[pos][3];
                    extend = data[pos][4];
                    intake= data[pos][5];
                    intake2= data[pos][6];

                    calcNextTargetPos(speedL,speedR, 1);

                    if((pos+1)<data.length) {
                        //pos++;
                    }

                    runtime.reset();
                }

                calcNextPos(hwmap.lw1.getCurrentPosition()-previousLeftPos,hwmap.rw1.getCurrentPosition()-previousRightPos,1);

                previousLeftPos = hwmap.lw1.getCurrentPosition();
                previousRightPos = hwmap.rw1.getCurrentPosition();

                errorR= setR-hwmap.rw1.getCurrentPosition();
                errorL = setL-hwmap.lw1.getCurrentPosition();
                double pr;
                double pl;


                double dErrorR = errorR - previousErrorR;
                double dErrorL = errorL - previousErrorL;

                double gyroError = gyro-hwmap.gyro.getHeading();

                //pr = errorR*Kp + Kd*dErrorR + Ki*sumErrorR + Kf*speedR + gyroError*Kg;
                //pl = errorL*Kp + Kd*dErrorL + Ki*sumErrorL + Kf*speedL - gyroError*Kg;

                double adjustedXError = Math.cos(sumTargetAngle)*(sumX - sumTargetX) - Math.sin(sumTargetAngle)*(sumY - sumTargetY);
                double adjustedYError = Math.sin(sumTargetAngle)*(sumX - sumTargetX) + Math.cos(sumTargetAngle)*(sumY - sumTargetY);

                double angleError = sumAngle - sumTargetAngle;

                //double forwardPower = -adjustedYError*0.05;
                //double anglePower = -adjustedXError*0.08 - angleError*0.8;

                if(adjustedYError < 0){
                    adjustedXError = -adjustedXError;
                }

                double forwardPower = -adjustedYError*0.1;
                double anglePower = -adjustedXError*0.1 - angleError*1;

                if(gamepad1.y){
                    pr = forwardPower + anglePower;
                    pl = forwardPower - anglePower;
                }else {
                    pr = 0;
                    pl = 0;
                }
                hwmap.rw1.setPower(pr);
                hwmap.rw2.setPower(pr);
                hwmap.lw1.setPower(pl);
                hwmap.lw2.setPower(pl);

                //telemetry.addData("pr",pr);
                //telemetry.addData("pl",pl);
                //telemetry.addData("errorL:", errorL);
                //telemetry.addData("errorR",errorR);
                telemetry.addData("adjustedXError", adjustedXError);
                telemetry.addData("adjustedYError", adjustedYError);
                telemetry.addData("sumTargetX:", sumTargetX);
                telemetry.addData("sumTargetY",sumTargetY);
                telemetry.addData("sumTargetAngle",sumTargetAngle);
                telemetry.addData("sumX:", sumX);
                telemetry.addData("sumY",sumY);
                telemetry.addData("sumAngle",sumAngle);

                csvData += setL + ","+hwmap.lw1.getCurrentPosition()+ "," + setR + ","+ hwmap.rw1.getCurrentPosition() +","+ sumTargetX +","+ sumTargetY +"," + sumTargetAngle + "," + Kf*speedL + "," + gyroError*Kg +"\r\n";

                sumErrorL += errorL;
                sumErrorR += errorR;

                if(sumErrorL > 1/Ki){
                    sumErrorL = (int) (1/Ki);
                }
                if(sumErrorR > 1/Ki){
                    sumErrorR = (int) (1/Ki);
                }


                previousErrorL = errorL;
                previousErrorR = errorR;


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

        String filename = "test.csv";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, csvData);
        telemetry.log().add("saved to '%s'", filename);

    }

    public void calcNextTargetPos(double leftSpeed, double rightSpeed, double timeStep){
        timeStep = 1;
        double driveBaseWidth = 14.4;

        double encoderTicksPerMotorRev = 28;
        double gearRatio = (1.0/9.0);
        double wheelDiameter = 2;

        double encoderTicksPerInch = (wheelDiameter * 3.1415)*encoderTicksPerMotorRev * (1.0/gearRatio);

        leftSpeed = ((double)leftSpeed/encoderTicksPerInch)*40.0;
        rightSpeed = ((double)rightSpeed/encoderTicksPerInch)*40.0;

        double centerSpeed = (leftSpeed + rightSpeed)/2.0;
        double centerDistance = centerSpeed * timeStep;

        double angleAdjust = 4;

        double changeX,changeY,changeA = 0;

        if(leftSpeed == rightSpeed){
            changeX = 0;
            changeY = centerDistance;
            changeA = 0;
        }else if(leftSpeed == -rightSpeed){
            changeX = 0;
            changeY  = 0;
            changeA = (-(leftSpeed*timeStep)/(driveBaseWidth))*angleAdjust;
        }else{
            double radius = ((rightSpeed + leftSpeed)/(rightSpeed - leftSpeed))*driveBaseWidth;
            changeX = (-radius * Math.cos(centerDistance/radius))+radius;
            changeY = radius * Math.sin(centerDistance/radius);
            changeA = (centerDistance/(radius*2))*angleAdjust;
        }

        sumTargetX += Math.cos(sumTargetAngle)*changeX + Math.sin(sumTargetAngle)*changeY;
        sumTargetY += Math.sin(sumTargetAngle)*changeX + Math.cos(sumTargetAngle)*changeY;

        sumTargetAngle += changeA;
    }

    public void calcNextPos(double leftSpeed, double rightSpeed, double timeStep){
        timeStep = 1;
        double driveBaseWidth = 14.4;

        double encoderTicksPerMotorRev = 28;
        double gearRatio = (1.0/9.0);
        double wheelDiameter = 2;

        double encoderTicksPerInch = (wheelDiameter * 3.1415)*encoderTicksPerMotorRev * (1.0/gearRatio);

        leftSpeed = ((double)leftSpeed/encoderTicksPerInch)*40.0;
        rightSpeed = ((double)rightSpeed/encoderTicksPerInch)*40.0;

        double centerSpeed = (leftSpeed + rightSpeed)/2.0;
        double centerDistance = centerSpeed * timeStep;

        double angleAdjust = 4;

        double changeX,changeY,changeA = 0;

        if(leftSpeed == rightSpeed){
            changeX = 0;
            changeY = centerDistance;
            changeA = 0;
        }else if(leftSpeed == -rightSpeed){
            changeX = 0;
            changeY  = 0;
            changeA = (-(leftSpeed*timeStep)/(driveBaseWidth))*angleAdjust;
        }else{
            double radius = ((rightSpeed + leftSpeed)/(rightSpeed - leftSpeed))*driveBaseWidth;
            changeX = (-radius * Math.cos(centerDistance/radius))+radius;
            changeY = radius * Math.sin(centerDistance/radius);
            changeA = (centerDistance/(radius*2))*angleAdjust;
        }

        sumX += Math.cos(sumAngle)*changeX + Math.sin(sumAngle)*changeY;
        sumY += Math.sin(sumAngle)*changeX + Math.cos(sumAngle)*changeY;

        sumAngle += changeA;
    }
}
