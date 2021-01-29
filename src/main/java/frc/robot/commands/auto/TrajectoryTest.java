package frc.robot.commands.auto;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BallSuck;
import frc.robot.subsystems.OldDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class TrajectoryTest extends SequentialCommandGroup {
    /**
     * @param turret The Turret Subsystem {@link Turret} so that we can shoot balls
     * @param ballsuck The BallSuck Subsystem {@link BallSuck} so that we can SUCC balls
     * @param drivetrain The Drivetrain Subsystem {@link OldDrivetrain} so that we can drive!
     * */
    public TrajectoryTest(Turret turret, Drivetrain drivetrain, BallSuck ballsuck){

        // Pose2d[] positions = {
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     new Pose2d(3, 0, new Rotation2d(0)),
        //     new Pose2d(6, 2, new Rotation2d(0)),
        //     new Pose2d(6, 5, new Rotation2d(0)),
        //     // new Pose2d(0,3, new Rotation2d(0)),
        //     // new Pose2d(0,0, new Rotation2d(0))
        // };
        Pose2d[] positions = {
            new Pose2d(0, 0, new Rotation2d(0)),
new Pose2d(0.000328751013597325, 0.00000041162816974299, new Rotation2d(0)),
new Pose2d(0.0079263639212369, 0.0000502711771116789, new Rotation2d(0)),
new Pose2d(0.0329799294849777, 0.000521880234801361, new Rotation2d(0)),
new Pose2d(0.0762335996034832, 0.00215426496640762, new Rotation2d(0)),
new Pose2d(0.150346472137812, 0.00548657191286001, new Rotation2d(0)),
new Pose2d(0.238328808592952, 0.00843932711049271, new Rotation2d(0)),
new Pose2d(0.353385774238268, 0.0135286766745398, new Rotation2d(0)),
new Pose2d(0.480745271256865, 0.0217965138493883, new Rotation2d(0)),
new Pose2d(0.63041716605466, 0.0320108476840209, new Rotation2d(0)),
new Pose2d(0.787005583135396, 0.0428071904161058, new Rotation2d(0)),
new Pose2d(0.953317599830539, 0.0532590486817881, new Rotation2d(0)),
new Pose2d(1.11685601384822, 0.0623497107442286, new Rotation2d(0)),
new Pose2d(1.28179929391221, 0.0704494061245055, new Rotation2d(0)),
new Pose2d(1.43846749617367, 0.0762911275729101, new Rotation2d(0)),
new Pose2d(1.59590969357862, 0.0808402878927807, new Rotation2d(0)),
new Pose2d(1.75013846484546, 0.0837549448067759, new Rotation2d(0)),
new Pose2d(1.89880286562699, 0.0847448339777841, new Rotation2d(0)),
new Pose2d(2.04505410383574, 0.0835150469827436, new Rotation2d(0)),
new Pose2d(2.18838763741916, 0.080258009317893, new Rotation2d(0)),
new Pose2d(2.33165841189031, 0.0740324401996159, new Rotation2d(0)),
new Pose2d(2.48334235925035, 0.0644131858248408, new Rotation2d(0)),
new Pose2d(2.63144867875869, 0.0499926568644714, new Rotation2d(0)),
new Pose2d(2.77442114064171, 0.0322885606021359, new Rotation2d(0)),
new Pose2d(2.93879721380553, 0.00850117481245918, new Rotation2d(0)),
new Pose2d(3.1106351921929, -0.0198426468104869, new Rotation2d(0)),
new Pose2d(3.21999191771833, -0.043555510256744, new Rotation2d(0)),
new Pose2d(3.30504901317945, -0.0686248896381723, new Rotation2d(0)),
new Pose2d(3.36377946151723, -0.0907771995056784, new Rotation2d(0)),
new Pose2d(3.40713019493118, -0.113640851042669, new Rotation2d(0)),
new Pose2d(3.45034406390839, -0.145749754048193, new Rotation2d(0)),
new Pose2d(3.52828100363384, -0.216311642716171, new Rotation2d(0)),
new Pose2d(3.6180136864948, -0.311705831020042, new Rotation2d(0)),
new Pose2d(3.72616632323955, -0.443974432563415, new Rotation2d(0)),
new Pose2d(3.80525044397791, -0.571392681458827, new Rotation2d(0)),
new Pose2d(3.85543432918997, -0.698050191822497, new Rotation2d(0)),
new Pose2d(3.8755470425095, -0.833188353452034, new Rotation2d(0)),
new Pose2d(3.87046392621357, -0.955367320893039, new Rotation2d(0)),
new Pose2d(3.8487382139173, -1.10767334640593, new Rotation2d(0)),
new Pose2d(3.8094322721197, -1.26434967501479, new Rotation2d(0)),
new Pose2d(3.7429546983592, -1.40859741558533, new Rotation2d(0)),
new Pose2d(3.6647772889196, -1.50820030739571, new Rotation2d(0)),
new Pose2d(3.57135836893859, -1.58146777932087, new Rotation2d(0)),
new Pose2d(3.47028420531335, -1.63085600251012, new Rotation2d(0)),
new Pose2d(3.33904649536467, -1.67331561837628, new Rotation2d(0)),
new Pose2d(3.17955265290688, -1.70387560875568, new Rotation2d(0)),
new Pose2d(2.98987549859209, -1.7206894875181, new Rotation2d(0)),
new Pose2d(2.7821721555523, -1.71779337340805, new Rotation2d(0)),
new Pose2d(2.5556798294379, -1.68702444176619, new Rotation2d(0)),
new Pose2d(2.31633296099995, -1.62396280016677, new Rotation2d(0)),
new Pose2d(2.07158191975025, -1.51991383988322, new Rotation2d(0)),
new Pose2d(1.86020198329063, -1.38268043242997, new Rotation2d(0)),
new Pose2d(1.7032744990136, -1.22382143477002, new Rotation2d(0)),
new Pose2d(1.612914595197, -1.05349558174114, new Rotation2d(0)),
new Pose2d(1.59271009321725, -0.894335336140806, new Rotation2d(0)),
new Pose2d(1.62478306652548, -0.758577887927144, new Rotation2d(0)),
new Pose2d(1.68768297328402, -0.635739923230769, new Rotation2d(0)),
new Pose2d(1.77134709441019, -0.519266409982132, new Rotation2d(0)),
new Pose2d(1.88671944992314, -0.395662076818687, new Rotation2d(0)),
new Pose2d(2.03005320770945, -0.275910160933588, new Rotation2d(0)),
new Pose2d(2.20363071515107, -0.159526048990702, new Rotation2d(0)),
new Pose2d(2.39792310757056, -0.0593183734879944, new Rotation2d(0)),
new Pose2d(2.65141670134368, 0.0398976094461618, new Rotation2d(0)),
new Pose2d(2.89899242255567, 0.112336492443505, new Rotation2d(0)),
new Pose2d(3.17983017292136, 0.173020385006323, new Rotation2d(0)),
new Pose2d(3.47632422363264, 0.21552188796376, new Rotation2d(0)),
new Pose2d(3.76443651920378, 0.239432123834867, new Rotation2d(0)),
new Pose2d(4.04005761510157, 0.253783480977261, new Rotation2d(0)),
new Pose2d(4.29061627187383, 0.258816737696209, new Rotation2d(0)),
new Pose2d(4.55020942778952, 0.25717633091553, new Rotation2d(0)),
new Pose2d(4.79727817887151, 0.25259314342142, new Rotation2d(0)),
new Pose2d(5.03097157929962, 0.250816277848244, new Rotation2d(0)),
new Pose2d(5.23160223229847, 0.256327823930077, new Rotation2d(0)),
new Pose2d(5.43497365199403, 0.266807320319652, new Rotation2d(0)),
new Pose2d(5.63018580407843, 0.281276749948032, new Rotation2d(0)),
new Pose2d(5.83491704814568, 0.300669086975737, new Rotation2d(0)),
new Pose2d(6.03773342483589, 0.326646383883467, new Rotation2d(0)),
new Pose2d(6.2142461350389, 0.36133000739576, new Rotation2d(0)),
new Pose2d(6.33311116693427, 0.402435852750313, new Rotation2d(0)),
new Pose2d(6.45899265296613, 0.470809041886446, new Rotation2d(0)),
new Pose2d(6.55850783949422, 0.539429019114875, new Rotation2d(0)),
new Pose2d(6.64997039385639, 0.620234927792909, new Rotation2d(0)),
new Pose2d(6.72395986905398, 0.708116583387053, new Rotation2d(0)),
new Pose2d(6.78660838930858, 0.806154558130421, new Rotation2d(0)),
new Pose2d(6.85199502830837, 0.931773125281609, new Rotation2d(0)),
new Pose2d(6.89931169109111, 1.0482319768415, new Rotation2d(0)),
new Pose2d(6.93083126342124, 1.16747229636807, new Rotation2d(0)),
new Pose2d(6.93713660496976, 1.25218451461365, new Rotation2d(0)),
new Pose2d(6.92432353463606, 1.33016538932502, new Rotation2d(0)),
new Pose2d(6.89834026385499, 1.40999468280737, new Rotation2d(0)),
new Pose2d(6.86491434630309, 1.5005667174073, new Rotation2d(0)),
new Pose2d(6.82372687189306, 1.60839047214891, new Rotation2d(0)),
new Pose2d(6.77575186095091, 1.71770562625362, new Rotation2d(0)),
new Pose2d(6.72363399064787, 1.80447894809002, new Rotation2d(0)),
new Pose2d(6.66652576821909, 1.87011619465195, new Rotation2d(0)),
new Pose2d(6.60354146652373, 1.92530954135159, new Rotation2d(0)),
new Pose2d(6.52658106628105, 1.97941179397094, new Rotation2d(0)),
new Pose2d(6.45552547015505, 2.02020524704914, new Rotation2d(0)),
new Pose2d(6.37957107610981, 2.05456183303883, new Rotation2d(0)),
new Pose2d(6.29066363103595, 2.08740319865282, new Rotation2d(0)),
new Pose2d(6.18469062140654, 2.12079780954447, new Rotation2d(0)),
new Pose2d(6.05419802934974, 2.15636027056512, new Rotation2d(0)),
new Pose2d(5.91645957614993, 2.1856831229617, new Rotation2d(0)),
new Pose2d(5.76788171029571, 2.21004107865678, new Rotation2d(0)),
new Pose2d(5.61425118672768, 2.22794873572516, new Rotation2d(0)),
new Pose2d(5.47892033975747, 2.23842962473235, new Rotation2d(0)),
new Pose2d(5.37259994434787, 2.24774138021674, new Rotation2d(0)),
new Pose2d(5.30684924642542, 2.24405964455454, new Rotation2d(0)),
new Pose2d(5.24143714158332, 2.22762978143516, new Rotation2d(0)),
new Pose2d(5.17911429717612, 2.20679888665956, new Rotation2d(0)),
new Pose2d(5.09290392482264, 2.17815462196996, new Rotation2d(0)),
new Pose2d(4.994519834149, 2.14275985736721, new Rotation2d(0)),
new Pose2d(4.87331131483824, 2.0865370194956, new Rotation2d(0)),
new Pose2d(4.73649638925492, 2.00917665320713, new Rotation2d(0)),
new Pose2d(4.58498833295542, 1.9076991125461, new Rotation2d(0)),
new Pose2d(4.45333349859106, 1.80560005300964, new Rotation2d(0)),
new Pose2d(4.41104487741115, 1.76524953674973, new Rotation2d(0)),
new Pose2d(4.37609808914123, 1.71749206908472, new Rotation2d(0)),
new Pose2d(4.34131459279629, 1.64405584643623, new Rotation2d(0)),
new Pose2d(4.3042829280875, 1.54287559793223, new Rotation2d(0)),
new Pose2d(4.26144954386274, 1.41254046746839, new Rotation2d(0)),
new Pose2d(4.2192355501374, 1.25752722259403, new Rotation2d(0)),
new Pose2d(4.18238880221258, 1.07834729456027, new Rotation2d(0)),
new Pose2d(4.15618134178519, 0.888607637839762, new Rotation2d(0)),
new Pose2d(4.13569874488993, 0.677635200550343, new Rotation2d(0)),
new Pose2d(4.14008649620594, 0.497301151484132, new Rotation2d(0)),
new Pose2d(4.16690902964154, 0.310841790787904, new Rotation2d(0)),
new Pose2d(4.21417784765985, 0.11422288268848, new Rotation2d(0)),
new Pose2d(4.27929121641652, -0.0782451498266519, new Rotation2d(0)),
new Pose2d(4.30751113261354, -0.13517599953337, new Rotation2d(0)),
new Pose2d(4.34621344868391, -0.173593564308995, new Rotation2d(0)),
new Pose2d(4.4512923023666, -0.253494299597297, new Rotation2d(0)),
new Pose2d(4.56819870543676, -0.328698806110153, new Rotation2d(0)),
new Pose2d(4.72269037480739, -0.419500717827276, new Rotation2d(0)),
new Pose2d(4.89112662578061, -0.507135128921474, new Rotation2d(0)),
new Pose2d(5.08342478671897, -0.595592478450487, new Rotation2d(0)),
new Pose2d(5.28326314873377, -0.676012564347671, new Rotation2d(0)),
new Pose2d(5.48769506980644, -0.747244669357576, new Rotation2d(0)),
new Pose2d(5.5867025116722, -0.77600070626489, new Rotation2d(0)),
new Pose2d(5.71265132201557, -0.805425362452836, new Rotation2d(0)),
new Pose2d(5.79582155142137, -0.818209799174121, new Rotation2d(0)),
new Pose2d(5.87701801167386, -0.827373854156219, new Rotation2d(0)),
new Pose2d(5.95771694698535, -0.835347463892047, new Rotation2d(0)),
new Pose2d(6.03645255859452, -0.842743533196322, new Rotation2d(0)),
new Pose2d(6.11676863976294, -0.849265068038474, new Rotation2d(0)),
new Pose2d(6.20652832218513, -0.855901538285798, new Rotation2d(0)),
new Pose2d(6.29853303409248, -0.862385608298489, new Rotation2d(0)),
new Pose2d(6.3976042004641, -0.868864322435915, new Rotation2d(0)),
new Pose2d(6.50548804458403, -0.875472243412018, new Rotation2d(0)),
new Pose2d(6.62060999268323, -0.882204340272167, new Rotation2d(0)),
new Pose2d(6.74479606189578, -0.889101363673712, new Rotation2d(0)),
new Pose2d(6.87188998887306, -0.895663071065829, new Rotation2d(0)),
new Pose2d(6.98710188233853, -0.900596501154325, new Rotation2d(0)),
new Pose2d(7.10229976643908, -0.903881052716949, new Rotation2d(0)),
new Pose2d(7.21928272925464, -0.903470413804756, new Rotation2d(0)),
new Pose2d(7.30743372125677, -0.895977814336035, new Rotation2d(0)),
new Pose2d(7.35803447922201, -0.886249124242746, new Rotation2d(0)),
new Pose2d(7.39556409561692, -0.876173142892931, new Rotation2d(0)),
new Pose2d(7.43915691308535, -0.862590833527702, new Rotation2d(0)),
new Pose2d(7.49359737554666, -0.845153315876988, new Rotation2d(0)),
new Pose2d(7.57415228874935, -0.81768674670404, new Rotation2d(0)),
new Pose2d(7.66469090335922, -0.783663709847162, new Rotation2d(0)),
new Pose2d(7.76795614871032, -0.741281112007893, new Rotation2d(0)),
new Pose2d(7.84053828695115, -0.705360164893017, new Rotation2d(0)),
new Pose2d(7.88366056455547, -0.67740891202521, new Rotation2d(0)),
new Pose2d(7.90975857630405, -0.655177556714007, new Rotation2d(0)),
new Pose2d(7.92947564669478, -0.634773740892229, new Rotation2d(0)),
new Pose2d(7.96351653626344, -0.595880424606745, new Rotation2d(0)),
new Pose2d(8.00374203808304, -0.548765653942835, new Rotation2d(0)),
new Pose2d(8.05584557614837, -0.485161160498156, new Rotation2d(0)),
new Pose2d(8.09253729995851, -0.435888023734187, new Rotation2d(0)),
new Pose2d(8.10064874947123, -0.424088957286115, new Rotation2d(0)),
new Pose2d(8.09696432155435, -0.429840638268688, new Rotation2d(0)),
new Pose2d(8.10853976305099, -0.411575255796269, new Rotation2d(0)),
new Pose2d(8.13860508605022, -0.362612556248623, new Rotation2d(0)),
new Pose2d(8.18847872769692, -0.273461274678932, new Rotation2d(0)),
new Pose2d(8.21924558085876, -0.203686940170319, new Rotation2d(0)),
new Pose2d(8.23309276521843, -0.139873874658499, new Rotation2d(0)),
new Pose2d(8.23334105229777, -0.0995796492018158, new Rotation2d(0)),
new Pose2d(8.22666614484429, -0.0650859960633389, new Rotation2d(0)),
new Pose2d(8.2134601133335, -0.0298549320935435, new Rotation2d(0)),
new Pose2d(8.18321670616031, 0.0294973129733851, new Rotation2d(0)),
new Pose2d(8.14150456447475, 0.102163029935808, new Rotation2d(0)),
new Pose2d(8.0927908872419, 0.175800778599789, new Rotation2d(0)),
new Pose2d(8.04702688500849, 0.228997322896888, new Rotation2d(0)),
new Pose2d(7.9930869624681, 0.272587203230808, new Rotation2d(0)),
new Pose2d(7.91910001279003, 0.312478590968851, new Rotation2d(0)),
new Pose2d(7.84413677403211, 0.343543587011667, new Rotation2d(0)),
new Pose2d(7.72829316107404, 0.383281683865671, new Rotation2d(0)),
new Pose2d(7.61857037583935, 0.417556541787911, new Rotation2d(0)),
new Pose2d(7.50396241069256, 0.455027856092626, new Rotation2d(0)),
new Pose2d(7.4134096736375, 0.485427669920806, new Rotation2d(0)),
new Pose2d(7.29317923377463, 0.522546829624742, new Rotation2d(0)),
new Pose2d(7.16879009168268, 0.546783753484988, new Rotation2d(0)),
new Pose2d(7.00784920137814, 0.562531149544061, new Rotation2d(0)),
new Pose2d(6.80441234616579, 0.572219810575606, new Rotation2d(0)),
new Pose2d(6.57432279243471, 0.575918995044108, new Rotation2d(0)),
new Pose2d(6.33106979606245, 0.573272133338365, new Rotation2d(0)),
new Pose2d(6.09990185300407, 0.56836959809447, new Rotation2d(0)),
new Pose2d(5.89049287293433, 0.568337106906076, new Rotation2d(0)),
new Pose2d(5.70567043811915, 0.572326170014494, new Rotation2d(0)),
new Pose2d(5.53075105417709, 0.578845010721583, new Rotation2d(0)),
new Pose2d(5.34160622561922, 0.587082262774443, new Rotation2d(0)),
new Pose2d(5.14285164128352, 0.596274512060275, new Rotation2d(0)),
new Pose2d(4.92359931831832, 0.608048546201915, new Rotation2d(0)),
new Pose2d(4.71349557202327, 0.624177772252194, new Rotation2d(0)),
new Pose2d(4.50467148253647, 0.644164174848031, new Rotation2d(0)),
new Pose2d(4.28119345587036, 0.668098658026765, new Rotation2d(0)),
new Pose2d(4.05007731364974, 0.691692399963636, new Rotation2d(0)),
new Pose2d(3.80232905027871, 0.715613282986087, new Rotation2d(0)),
new Pose2d(3.54015484610903, 0.739052175194314, new Rotation2d(0)),
new Pose2d(3.2738193791182, 0.760044879435243, new Rotation2d(0)),
new Pose2d(3.00480960635415, 0.776282765521129, new Rotation2d(0)),
new Pose2d(2.74596814025531, 0.786741708859157, new Rotation2d(0)),
new Pose2d(2.50013263566122, 0.797023869852633, new Rotation2d(0)),
new Pose2d(2.27287516938604, 0.808202083408338, new Rotation2d(0)),
new Pose2d(2.05831631080375, 0.820188046851446, new Rotation2d(0)),
new Pose2d(1.85058808656782, 0.833197588416591, new Rotation2d(0)),
new Pose2d(1.65064572943056, 0.848144482301689, new Rotation2d(0)),
new Pose2d(1.44837440860944, 0.865150653434733, new Rotation2d(0)),
new Pose2d(1.24094732018273, 0.88317627066807, new Rotation2d(0)),
new Pose2d(1.02984671158788, 0.900235022576137, new Rotation2d(0)),
new Pose2d(0.812269601414093, 0.916450362887069, new Rotation2d(0)),
new Pose2d(0.590797398152825, 0.931455310660517, new Rotation2d(0)),
new Pose2d(0.370100723930916, 0.944847882720963, new Rotation2d(0)),
new Pose2d(0.160893494212768, 0.955680001560215, new Rotation2d(0)),
new Pose2d(-0.0329603996115828, 0.965631993904447, new Rotation2d(0)),
new Pose2d(-0.196822261750002, 0.972592513053972, new Rotation2d(0)),
new Pose2d(-0.330270380854804, 0.977703771036643, new Rotation2d(0)),
new Pose2d(-0.438673591797967, 0.981989791108601, new Rotation2d(0)),
new Pose2d(-0.522773360598083, 0.98517951164212, new Rotation2d(0)),
new Pose2d(-0.585159094996967, 0.987426390148681, new Rotation2d(0)),
new Pose2d(-0.619290486380881, 0.988657789580207, new Rotation2d(0)),
new Pose2d(-0.630022068642688, 0.989062400558094, new Rotation2d(0)),
new Pose2d(-0.629182516988078, 0.989030908252341, new Rotation2d(0))
        };
        addCommands(new TrajectoryFollow(drivetrain, positions));
        
            // Create a voltage constraint to ensure we don't accelerate too fast
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10.0);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint);


        //First test to see if it can follow a basic path
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.1518347624, 0.0532079543, new Rotation2d(7.799128318 * 3.1415926535/180.0)),
            List.of(
                new Translation2d(0,3),
                new Translation2d(1,10)
            ),
            new Pose2d(2, 15, new Rotation2d(4.410397358 * 3.1415926535/180.0)),
            config
        );

        // addCommands(new FollowTrajectory(exampleTrajectory, drivetrain));


        //Using a simple path generated in Pathweaver:
        String testJson = "paths/Barell.wpilib.json";
        Trajectory testTrajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(testJson);
            testTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + testJson, ex.getStackTrace());
        }
        // addCommands(new FollowTrajectory(testTrajectory, drivetrain));

        String barell2Json = "paths/Barell2.wpilib.json";
        Trajectory barell2Trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(barell2Json);
            barell2Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + testJson, ex.getStackTrace());
        }
        // addCommands(new FollowTrajectory(barell2Trajectory, drivetrain));
        
        String barell3Json = "paths/Barell3.wpilib.json";
        Trajectory barell3Trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(barell3Json);
            barell3Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + testJson, ex.getStackTrace());
        }
        // addCommands(new FollowTrajectory(barell3Trajectory, drivetrain));
        
        //The barell path generated in Pathweaver:
        /* String barellJson = "paths/Test.wpilib.json";
        Trajectory barellTrajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(barellJson);
            barellTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + barellJson, ex.getStackTrace());
        }
        drivetrain.resetOdometry(barellTrajectory.getInitialPose());
        addCommands(new FollowTrajectory(barellTrajectory, drivetrain)); */
    }
}