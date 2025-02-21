// https://mvnrepository.com/artifact/fr.brouillard.oss/jgitver
@Grapes(
@Grab(group='fr.brouillard.oss', module='jgitver', version='0.14.0')
)
import fr.brouillard.oss.jgitver.*;
import eu.mihosoft.vrl.v3d.*;
import javafx.scene.text.Font;

import com.google.gson.reflect.TypeToken
import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.creature.IgenerateBed
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
//import com.neuronrobotics.bowlerstudio.vitamins.VitaminBomManager
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.ChamferedCube
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.FileUtil
import eu.mihosoft.vrl.v3d.PrepForManufacturing
import eu.mihosoft.vrl.v3d.RoundedCube
import eu.mihosoft.vrl.v3d.Sphere
import eu.mihosoft.vrl.v3d.Transform
import eu.mihosoft.vrl.v3d.parametrics.LengthParameter
import javafx.scene.paint.Color
import javafx.scene.transform.Affine
import eu.mihosoft.vrl.v3d.ChamferedCylinder
import java.lang.reflect.Type
import java.nio.file.Paths

import javax.xml.transform.TransformerFactory

import org.apache.commons.math3.analysis.function.Atan
import org.apache.commons.math3.analysis.function.Sqrt
import org.apache.commons.math3.genetics.GeneticAlgorithm

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.neuronrobotics.bowlerstudio.creature.MobileBaseCadManager
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.addons.kinematics.VitaminLocation;
import javafx.application.Platform


CSG ChamferedCylinder(double r, double h, double chamferHeight) {
	CSG cube1 = new Cylinder(r - chamferHeight,r - chamferHeight, h,40).toCSG();
	CSG cube2 = new Cylinder(r,r, h - chamferHeight * 2,40).toCSG().movez(chamferHeight);
	return cube1.union(cube2).hull()
}
double computeGearPitch(double diameterAtCrown,double numberOfTeeth){
	return ((diameterAtCrown/2)*((360.0)/numberOfTeeth)*Math.PI/180)
}
// Load the devices to map the kinematics of the gear wrist
// this is loaded here in case in the future we need to pass the gear ratio to the gear wrist kinematics.
//ScriptingEngine.gitScriptRun("https://github.com/OperationSmallKat/Marcos.git", "GearWristKinematics.groovy")

url = "https://github.com/OperationSmallKat/Marcos.git"
File parametricsCSV = ScriptingEngine.fileFromGit(url, "parametrics.csv")
HashMap<String,Double> numbers;
BufferedReader reader;
String code="HashMap<String,Double> numbers = new HashMap<>()\n"
String vars=""
String equs=""
try {
	reader = new BufferedReader(new FileReader(parametricsCSV.getAbsolutePath()));
	String line = reader.readLine();
	while ((line = reader.readLine() )!= null) {
		if(line.length()>3) {
			//System.out.println(line);
			String[] parts = line.split(",");
			String value=(parts[2].replaceAll(parts[1], "")).trim()
			String reconstructed =parts[0]+"="+value;
			try {
				Double.parseDouble(value)
				vars+= reconstructed+"\n"
				vars+="numbers.put(\""+parts[0]+"\","+parts[0]+");\n"
			}catch(NumberFormatException ex) {
				equs+= reconstructed+"\n"
				equs+="numbers.put(\""+parts[0]+"\","+parts[0]+");\n"
			}
		}
	}
	reader.close();
} catch (IOException e) {
	e.printStackTrace();
}
code+=vars;
code+=equs;
code+="return numbers"
//println code
numbers=(HashMap<String,Double>) ScriptingEngine.inlineScriptStringRun(code, null, "Groovy");
//for(String key :numbers.keySet()) {
//	println key+" : "+numbers.get(key)
//}
def honrConfig = Vitamins.getConfiguration("hobbyServoHorn", "standardMicro1")
double hornDiam = honrConfig.hornBaseDiameter

// Begin creating the resin print horn piece withn a chamfered cylendar
CSG core=  ChamferedCylinder(hornDiam/2.0,hornDiam,numbers.Chamfer2)
//calculate the depth of the screw head based on the given measurments
double cutoutDepth = numbers.ServoHornHeight-numbers.ServoMountingScrewSpace - numbers.ServoHornSplineHeight
// the cutout for the head of the screw on the resin horn
CSG screwHeadCutOut = new Cylinder(numbers.ServoHornScrewHeadDiamter/2.0,numbers.ServoHornScrewHeadDiamter/2.0, cutoutDepth,30).toCSG()
		.toZMax()
		.movez(numbers.ServoHornHeight)
// cutout for the hole the shaft of the mount screw passes through
CSG screwHoleCutOut = new Cylinder(numbers.ServoHornScrewDiamter/2.0,numbers.ServoHornScrewDiamter/2.0, numbers.ServoMountingScrewSpace,30).toCSG()
		.toZMax()
		.movez(numbers.ServoHornHeight-cutoutDepth)
// Cut the holes from the core
CSG cutcore=core.difference([
	screwHeadCutOut,
	screwHoleCutOut
])
double spineDiameter=4.68+0.2


// use the gear maker to generate the spline
//def gears = ScriptingEngine.gitScriptRun(
//		"https://github.com/madhephaestus/GearGenerator.git", // git location of the library
//		"bevelGear.groovy" , // file to load
//		// Parameters passed to the funcetion
//		[
//			numbers.ServoHornNumberofTeeth,
//			// Number of teeth gear a
//			numbers.ServoHornNumberofTeeth,
//			// Number of teeth gear b
//			numbers.ServoHornSplineHeight,
//			// thickness of gear A
//			computeGearPitch(spineDiameter,numbers.ServoHornNumberofTeeth),
//			// gear pitch in arc length mm
//			0,
//			// shaft angle, can be from 0 to 100 degrees
//			0// helical angle, only used for 0 degree bevels
//		]
//		)
//// get just the pinion of the set
//CSG spline = gears.get(0)
// cut the spline from the core
CSG resinPrintServoMount=cutcore//.difference(spline)
resinPrintServoMount.setColor(Color.DARKGREY)
resinPrintServoMount.setName("ResinHorn")
//return resinPrintServoMount
class cadGenMarcos implements ICadGenerator{
	String url = "https://github.com/OperationSmallKat/Marcos.git"
	LengthParameter offset		= new LengthParameter("printerOffset",0.0,[2, 0])
	
	CSG resinPrintServoMount
	HashMap<String,Double> numbers
	LengthParameter tailLength		= new LengthParameter("Cable Cut Out Length",30,[500, 0.01])
	double endOfPassiveLinkToBolt = 4.5
	double hornDiam;
	Gson gson = new GsonBuilder().disableHtmlEscaping().setPrettyPrinting().create();
	//VitaminBomManager bom;
	CSG cachedGearLink =null;

	public cadGenMarcos(CSG res,HashMap<String,Double> n,double h) {
		resinPrintServoMount=res
		numbers=n
		hornDiam=h
	}
	ArrayList<CSG> cache = new ArrayList<CSG>()
	private MobileBase mb;
	CSG moveDHValues(CSG incoming,DHParameterKinematics d, int linkIndex ){
		TransformNR step = new TransformNR(d.getChain().getLinks().get(linkIndex).DhStep(0)).inverse()
		Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
		return incoming.transformed(move)
	}
	CSG reverseDHValues(CSG incoming,DHParameterKinematics d, int linkIndex ){
		TransformNR step = new TransformNR(d.getChain().getLinks().get(linkIndex).DhStep(0))
		Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
		return incoming.transformed(move)
	}

	CSG footBallSection() {
		def capThickness=2.25
		def ballRadius = 10
		def radius = ballRadius-(capThickness/2.0)
		def neckRad = 6
		def arclen=16.5
		def neckThicknes =3.5
		def theta = (arclen*360)/(2.0*3.14159*radius)
		def internalAngle = (90-(theta/2))
		def d = Math.sin(Math.toRadians(internalAngle))*radius

		//println d +" "+theta+" cir="+(3.14159*radius)+ " ind angle="+internalAngle

		CSG slicer = new Cylinder(radius*2, neckThicknes).toCSG()
				.difference(new Cylinder(neckRad,neckRad+neckThicknes, neckThicknes,15).toCSG())
				.toZMax()
				.movez(d)
		CSG slicer2 = new Cylinder(radius*2, radius*2).toCSG()

		CSG foot = new Sphere(ballRadius,32, 16).toCSG()
				.difference(slicer2.movez(d-neckThicknes))

		CSG ball  = new Sphere(radius,32, 16).toCSG()
				.difference(slicer)
				//.difference(slicer2)
				.union(foot)
		//.union(new Cylinder(radius-2, ballRadius).toCSG().toZMax())
	}
	CSG toBed(ArrayList<CSG> parts, String name) {
		CSG bedOne=null
		for(CSG p:parts) {
			if(bedOne==null)
				bedOne=p
			else {
				bedOne=bedOne.dumbUnion(p)
			}
		}
		if(bedOne!=null)
			bedOne.setName(name)
		else {
			bedOne = new Cube().toCSG()
			bedOne.setManufacturing({return null})
		}

		return bedOne
	}
	CSG ChamferedCylinder(double r, double h, double chamferHeight) {
		CSG c1 = new Cylinder(r - chamferHeight,r - chamferHeight, h,40).toCSG()
		CSG c2 = new Cylinder(r,r, h - chamferHeight * 2,40).toCSG().movez(chamferHeight)
		return c1.union(c2).hull()
	}
	CSG ChamferedCylinderHR(double r, double h, double chamferHeight) {
		CSG c1 = new Cylinder(r - chamferHeight,r - chamferHeight, h,80).toCSG()
		CSG c2 = new Cylinder(r,r, h - chamferHeight * 2,80).toCSG().movez(chamferHeight)
		return c1.union(c2).hull()
	}
	CSG StraightChamfer(double x, double y, double chamferHeight) {
		CSG c1 = new Cube(x, y, chamferHeight).toCSG().movez(chamferHeight)
		CSG c2 = new Cube(x+chamferHeight,y+chamferHeight, chamferHeight).toCSG()
		CSG c3 = c1.union(c2).hull()
		return c3.difference(c2).toZMin()
	}
	CSG StraightChamfer(double x, double y,double z, double xycham,double zcham) {
		CSG lower = new ChamferedCube(x, y, z+xycham*2, xycham).toCSG()
		lower = lower.intersect(lower.getBoundingBox().movez(xycham))
		lower = lower.intersect(lower.getBoundingBox().movez(-xycham))
				.movez(-xycham+zcham)

		CSG upper = new ChamferedCube(x, y, z, zcham).toCSG()
		return new ChamferedCube(x, y, z, zcham).toCSG()
	}
	CSG ChamferedRoundCornerLug(double x, double y,double r, double h, double chamferHeight) {
		CSG corners = ChamferedCylinder(r,h,chamferHeight)
		CSG xSec= corners.union(corners.movex(x-r*2))
		return xSec.union(xSec.movey(y-(r*2))).hull().toXMin().toYMin().movex(-x/2).movey(-y/2)
	}
	public CSG calibrationLink(double rotationCenterToBoltCenter,CSG bolt) {
		CSG core= linkCore(rotationCenterToBoltCenter, bolt,numbers.LooseTolerance)
		double defaultValue = numbers.LinkLength - endOfPassiveLinkToBolt
		CSG stl= Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"DriveLink.stl"))
		double chamfer = numbers.Chamfer2
		double smallChamfer = numbers.Chamfer1/1.5
		double linkWidth = numbers.LinkWidth
		double linkRadius = linkWidth/2
		double linkThickness = numbers.LinkHeight
		double filletRad=numbers.Fillet3
		double LinkMountingCutOutWidth=numbers.LinkMountingCutOutWidth
		double blockx=rotationCenterToBoltCenter-numbers.LinkMountingCutOutLength-numbers.Tolerance+endOfPassiveLinkToBolt+filletRad
		double ServoHornRad=(hornDiam+numbers.ServoHornHoleTolerance/2)/2.0
		double ServoHornHeight =numbers.ServoHornHeight+numbers.LooseTolerance
		double mountHeadRad =( numbers.MountingScrewHeadDiamter+numbers.LooseTolerance)/2.0
		double mountRad=(numbers.MountingScrewDiamter+numbers.LooseTolerance)/2.0
		double decritiveRad = numbers.ServoHornDiameter/4.0
		double SetscrewLength = numbers.SetScrewLength
		double SetscrewSize = numbers.SetScrewSize
		double SquareNutWidth = numbers.SquareNutWidth + numbers.LooseTolerance
		double SquareNutHeight = numbers.SquareNutHeight + numbers.LooseTolerance
		double SquareNutCutOutHeight = linkThickness/2+SquareNutWidth/2
		double LinkSqaureNutSpacing = numbers.LinkSqaureNutSpacing-3

		//Solving for Angle of setscrew.
		double hypot1 = Math.hypot(ServoHornRad + SetscrewLength + numbers.LooseTolerance, SetscrewSize/2)
		double angle1 = Math.asin(linkRadius/hypot1)
		double angle2 = Math.asin((ServoHornRad + SetscrewLength + numbers.LooseTolerance)/hypot1)
		double angle3 = (Math.PI/2)-angle1
		double angle4 = (Math.PI/2)-angle2
		double SetScrewAngle = Math.toDegrees((Math.PI/2)-(angle3+angle4))
		double SetScrewChamferLength = linkRadius/Math.sin((Math.PI/2)-(angle3+angle4))
		double SetScrewCutOutLength = numbers.LinkLength/Math.cos((Math.PI/2)-(angle3+angle4))


		println(SetScrewChamferLength)
		println(SetScrewCutOutLength)

		CSG screwHole = new Cylinder(4.2/2.0, core.getTotalZ()).toCSG()

		CSG ServoHornCutoutChamfer = ChamferedCylinder(ServoHornRad+smallChamfer,ServoHornHeight+smallChamfer,smallChamfer)
				.toZMax()
				.movez(smallChamfer)
		// Idle pin cutout
		CSG ServoHornCutout = ChamferedCylinder(ServoHornRad,ServoHornHeight,smallChamfer)
				//.movez(-smallChamfer)
				.union(ServoHornCutoutChamfer)
				.union(screwHole)
		double distanceOfSquareNut = (LinkSqaureNutSpacing+linkRadius)-(SquareNutHeight/2)
		CSG SquareNutCutOut = new Cube(SquareNutHeight,SquareNutWidth, SquareNutCutOutHeight).toCSG()
				.toZMin()
				.movex(distanceOfSquareNut)

		CSG SquareNutChamfer = StraightChamfer(SquareNutHeight,SquareNutWidth,smallChamfer)
				.movex(distanceOfSquareNut)

		CSG SetScrewCutOut = new Cylinder(SetscrewSize/2, SetScrewCutOutLength).toCSG()
				.toZMin()
				.roty(-90)
				.movez(linkThickness/2)

		//Chamfer for set screw
		CSG cutout1 = new Cylinder((SetscrewSize)/2, SetScrewCutOutLength).toCSG()
				.toZMax()
				.roty(90)
				.movez(linkThickness/2)
				.rotz(SetScrewAngle)

		CSG cutout2 = new Cylinder((SetscrewSize)/2+chamfer, SetScrewCutOutLength/2).toCSG()
				.toZMax()
				.roty(90)
				.movez(linkThickness/2)
				.movex(linkRadius)
				.rotz(SetScrewAngle)

		CSG Flatwall = new Cube(numbers.LinkLength,linkWidth,linkThickness).toCSG()
				.toZMax()
				.movez(linkThickness)
				.movex(numbers.LinkLength/2)
		CSG Flatwall2 = new Cube(numbers.LinkLength,linkWidth,linkThickness).toCSG()
				.toZMax()
				.movez(linkThickness)
				.movex(numbers.LinkLength/2)
				.movey(chamfer)

		CSG Flatwall3 = new Cube(numbers.LinkLength,linkWidth,linkThickness).toCSG()
				.toZMax()
				.movez(linkThickness)
				.movex(numbers.LinkLength/2)
				.movey(-linkWidth)
				.movey(-chamfer)

		CSG p1 = ((Flatwall.intersect(cutout1)).difference(Flatwall2))
		CSG p2 = (cutout2.difference(Flatwall).difference(Flatwall3))

		CSG SetScrewChamferleft = p2.union(p1).hull()
		CSG SetScrewChamferright = SetScrewChamferleft.mirrory()


		// Assemble the whole link
		CSG link = core
				.difference(ServoHornCutout)
				.difference(SquareNutCutOut.rotz(SetScrewAngle))
				.difference(SquareNutCutOut.rotz(-SetScrewAngle))
				.difference(SetScrewCutOut.rotz(SetScrewAngle))
				.difference(SetScrewCutOut.rotz(-SetScrewAngle))
				.difference(SquareNutChamfer.rotz(SetScrewAngle))
				.difference(SetScrewChamferleft)
				.difference(SetScrewChamferright)
		//link.setIsWireFrame(true)
		link.setColor(Color.DARKRED)
		return link//.union(stl)

		//return stl
	}

	public CSG linkCore(double rotationCenterToBoltCenter,CSG bolt,double tollerence) {
		double defaultValue = numbers.LinkLength - endOfPassiveLinkToBolt
		double chamfer = numbers.Chamfer2
		double smallChamfer = numbers.Chamfer1
		double linkWidth = numbers.LinkWidth
		double linkThickness = numbers.LinkHeight
		double filletRad=numbers.Fillet3
		double LinkMountingCutOutWidth=numbers.LinkMountingCutOutWidth
		double blockx=rotationCenterToBoltCenter-numbers.LinkMountingCutOutLength-numbers.Tolerance+endOfPassiveLinkToBolt+filletRad
		double IdlePinRad=(numbers.IdlePinDiamter+tollerence)/2.0
		double idlePinHeight  =numbers.IdlePinThickness+tollerence
		double mountHeadRad =( numbers.MountingScrewHeadDiamter+tollerence)/2.0
		double mountRad=( numbers.MountingScrewDiamter+tollerence)/2.0
		double decritiveRad = numbers.ServoHornDiameter/4.0
		double zipTieLugDepth = 4
		double zipTieWidth=3
		double zipTieLugDIstanceFromEnd = 3.7
		double zipTieClerence =1.2
		double zipTieLugX=rotationCenterToBoltCenter-endOfPassiveLinkToBolt-zipTieLugDIstanceFromEnd
		// Hull together a toolshape to make the cutter to make the shape appropratly
		CSG cornerFilletCutter = new Cylinder(filletRad, linkThickness, 30).toCSG()
		// cut from the corner to the ege of the link
		cornerFilletCutter=cornerFilletCutter.union(cornerFilletCutter.movey(LinkMountingCutOutWidth)).hull()
		// cut from the corner to the end of where the fillet should end
		cornerFilletCutter=cornerFilletCutter.union(cornerFilletCutter.movex(chamfer)).hull()
		CSG leftCorner = cornerFilletCutter.movex(blockx).movey(linkWidth/2-LinkMountingCutOutWidth+filletRad)
		CSG rightCorner = cornerFilletCutter.movex(blockx).movey(-linkWidth/2-filletRad)

		CSG lowerEnd = ChamferedCylinder(linkWidth/2, linkThickness, chamfer)
		CSG linkBlock = new ChamferedCube(blockx+chamfer, linkWidth, linkThickness, chamfer).toCSG()
				.toZMin()
				.toXMin()
		// Trim the end chamfer off the end of the link block to make the end flat
		linkBlock=linkBlock.intersect(linkBlock.getBoundingBox().movex(-chamfer))
		// Use chamferd cylendars to make the lug at the end of the link
		CSG mountLug = ChamferedRoundCornerLug(blockx, linkWidth-(LinkMountingCutOutWidth*2),filletRad, linkThickness+chamfer, chamfer)
				.toZMin()
				.toXMax()
		// Make a champfered cylendar to make the inner chamfer radius'
		CSG LowerInnerCornerChamferCutLeft= ChamferedCylinder(filletRad+chamfer, chamfer*2+1, chamfer)
				.movex(blockx)
				.movey(linkWidth/2-LinkMountingCutOutWidth+filletRad)
				.toZMax()
				.movez(chamfer)
		LowerInnerCornerChamferCutLeft=LowerInnerCornerChamferCutLeft.union(LowerInnerCornerChamferCutLeft.movey(LinkMountingCutOutWidth)).hull()
		CSG LowerInnerCornerChamferCutRight = LowerInnerCornerChamferCutLeft.movey(-linkWidth)
		// trim off the top chamfers and mofe the block end to the tip of the link block
		mountLug=mountLug.difference(mountLug.getBoundingBox().movez(linkThickness))
				.movex(rotationCenterToBoltCenter+endOfPassiveLinkToBolt)


		CSG MountHeadHoleCutoutChamfer = ChamferedCylinder(mountHeadRad+smallChamfer,linkThickness+smallChamfer,smallChamfer)
				.toZMin()
				.movez(linkThickness-smallChamfer)
		CSG MountHoleCutoutChamfer = ChamferedCylinder(mountRad+smallChamfer,linkThickness+smallChamfer,smallChamfer)
				.toZMax()
				.movez(smallChamfer)


		CSG mountAssebmbly = MountHoleCutoutChamfer
				//.union(MountHeadHoleCutoutChamfer)
				.movex(rotationCenterToBoltCenter)
		if(bolt!=null) {
			CSG boltHole = bolt.toZMax()
					.movez(linkThickness)
			mountAssebmbly=mountAssebmbly
					.union(boltHole)
		}
		CSG decritiveDivit = ChamferedCylinder(decritiveRad+chamfer,chamfer*2+1,chamfer)
				.movez(linkThickness-chamfer)
		CSG decoration = decorationGen(rotationCenterToBoltCenter)

		// Assemble the whole link
		CSG link = lowerEnd
				.union(linkBlock)
				.hull()
				.union(mountLug)
				.difference(leftCorner)
				.difference(rightCorner)
				.difference(LowerInnerCornerChamferCutRight)
				.difference(LowerInnerCornerChamferCutLeft)
				.difference(decritiveDivit)
				.difference(decoration)
		if(bolt!=null)
			link=link.difference(mountAssebmbly)
		//link.setIsWireFrame(true)
		link.setColor(Color.DARKRED)
		return link//.union(stl)
	}

	public CSG passiveLink(double rotationCenterToBoltCenter,CSG bolt) {
		CSG core= linkCore(rotationCenterToBoltCenter, bolt,numbers.LooseTolerance)

		double defaultValue = numbers.LinkLength - endOfPassiveLinkToBolt
		CSG stl= Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"IdleLinkLeg.stl"))
		double chamfer = numbers.Chamfer2
		double smallChamfer = numbers.Chamfer1
		double linkWidth = numbers.LinkWidth
		double linkThickness = numbers.LinkHeight
		double filletRad=numbers.Fillet3
		double LinkMountingCutOutWidth=numbers.LinkMountingCutOutWidth
		double blockx=rotationCenterToBoltCenter-numbers.LinkMountingCutOutLength-numbers.Tolerance+endOfPassiveLinkToBolt+filletRad
		double IdlePinRad=(numbers.IdlePinDiamter+numbers.LooseTolerance)/2.0
		double idlePinHeight  =numbers.IdlePinThickness+numbers.LooseTolerance
		double mountHeadRad =( numbers.MountingScrewHeadDiamter+numbers.LooseTolerance)/2.0
		double mountRad=( numbers.MountingScrewDiamter+numbers.LooseTolerance)/2.0
		double decritiveRad = numbers.ServoHornDiameter/4.0
		double zipTieLugDepth = 4
		double zipTieWidth=3
		double zipTieLugDIstanceFromEnd = 3.7
		double zipTieClerence =1.2
		double zipTieLugX=rotationCenterToBoltCenter-endOfPassiveLinkToBolt-zipTieLugDIstanceFromEnd

		CSG IdlePinCutoutChamfer = ChamferedCylinder(IdlePinRad+smallChamfer,idlePinHeight+smallChamfer,smallChamfer)
				.toZMax()
				.movez(smallChamfer)
		// Idle pin cutout
		CSG IdlePinCutout = ChamferedCylinder(IdlePinRad,idlePinHeight,smallChamfer)
				//.movez(-smallChamfer)
				.union(IdlePinCutoutChamfer)



		CSG zipLug = new RoundedCube(zipTieWidth+chamfer*2,zipTieLugDepth-zipTieClerence,linkThickness-(zipTieClerence*2))
				.cornerRadius(chamfer)
				.toCSG()
				.toZMin()
				.toXMax()
				.toYMax()
				.movex(chamfer)
				.movez(zipTieClerence)
		CSG zipTieCut = new Cube(zipTieWidth,zipTieLugDepth,linkThickness).toCSG()
				.toZMin()
				.toXMax()
				.toYMax()
				.difference(zipLug)
		CSG bottomChamfer = new ChamferedCube(zipTieWidth+smallChamfer*2,zipTieLugDepth+smallChamfer*2,linkThickness,smallChamfer).toCSG()
				.toZMax()
				.toXMax()
				.toYMax()
				.movez(smallChamfer)
				.movex(smallChamfer)
				.movey(smallChamfer)
		zipTieCut=zipTieCut.union(bottomChamfer)
		zipTieCut=zipTieCut.movey(linkWidth/2)
				.movex(zipTieLugX)
		CSG rightZipTie=zipTieCut.mirrory()

		// Assemble the whole link
		CSG link = core
				.difference(IdlePinCutout)
				.difference(zipTieCut)
				.difference(rightZipTie)
		//link.setIsWireFrame(true)
		link.setColor(Color.DARKRED)
		return link//.union(stl)
	}


	CSG decorationGen(double rotationCenterToBoltCenter) {
		double backOffset = 4

		double chamfer = numbers.Chamfer2

		double x=rotationCenterToBoltCenter+chamfer-numbers.LinkDetailSize/2-backOffset-endOfPassiveLinkToBolt-0.5
		double y=numbers.LinkWidth-numbers.LinkDetailSize*2+chamfer*2
		double filletRad=numbers.Fillet3
		CSG smallCut=ChamferedCylinder((numbers.LinkWidth-numbers.LinkDetailSize*2)/2, chamfer*2+1, chamfer)
				.toZMax()
				.movez(chamfer)
		CSG largeCut=ChamferedCylinder(numbers.LinkWidth/2, chamfer*2+1, chamfer)
				.toZMax()
				.movez(chamfer)
				.movex(rotationCenterToBoltCenter)
		CSG lug = ChamferedRoundCornerLug(x,y,filletRad,chamfer*2+1,chamfer)
				.toXMin()
				.movex(-chamfer+backOffset)
				.difference(largeCut)
				.difference(smallCut)
				.movez(numbers.LinkHeight-chamfer)


		return lug
	}

	def getGearLink() {
		if(cachedGearLink==null) {
			// generate the link
			double gearDiameter = 27.65
			cachedGearLink= Vitamins.get(ScriptingEngine.fileFromGit(
					"https://github.com/OperationSmallKat/Marcos.git",
					"DriveGear.stl"))
					.moveToCenterX()
					.moveToCenterY()
					.rotz(180)
					.movez(0.15)

			double ServoHornRad=(6.96+0.2)/2.0
			double ServoHornDepth=5.15
			double squareNutDepth = 2+ServoHornRad
			double setScrewRadius = 3.3/2.0
			
			double smallChamfer = numbers.Chamfer1
			CSG stl = cachedGearLink
			double setScrewLen = gearDiameter/2;
			double lowerSectionHeight =Math.abs(stl.getMinZ())
			double previousOffset = offset.getMM()
			double nutDepth = -lowerSectionHeight+ServoHornDepth/2
			offset.setMM(0.2);
			CSG nutStart = Vitamins.get("squareNut", "M3")
					.roty(-90)
			offset.setMM(previousOffset);
					
			CSG nutChamfer = new ChamferedCube(nutStart.getTotalX()+smallChamfer,
				 nutStart.getTotalY()+smallChamfer,
				 3*smallChamfer, smallChamfer).toCSG()
				 .toZMax()
				 .movex(nutStart.getTotalX()/2)
				 .movez(-lowerSectionHeight+smallChamfer)
			CSG squareNut = nutStart
					.movez(nutDepth)
					.union(nutChamfer)
					.movex(squareNutDepth)
			
			CSG fill = ChamferedCylinder(setScrewLen, lowerSectionHeight+smallChamfer,smallChamfer)
					.toZMax()
					.movez(smallChamfer)
			fill=fill.difference(fill.getBoundingBox().toZMin())
			CSG setScrewChamfer =  ChamferedCylinder(setScrewRadius+smallChamfer, 3*smallChamfer,smallChamfer)
									.movez(setScrewLen-smallChamfer)
			CSG setScrew = new Cylinder(setScrewRadius, setScrewLen).toCSG()
					.union(setScrewChamfer)
					.roty(-90)
					.movez(-lowerSectionHeight+ServoHornDepth/2)
			CSG ServoHornCutoutChamfer = ChamferedCylinder(ServoHornRad+smallChamfer,3*smallChamfer,smallChamfer)
					.toZMax()
					.movez(smallChamfer)
			// Idle pin cutout
			CSG ServoHornCutout = ChamferedCylinder(ServoHornRad,ServoHornDepth,smallChamfer)
					.union(ServoHornCutoutChamfer)
					.movez(-lowerSectionHeight)
			CSG asm= fill.difference(setScrew)
					.difference(setScrew.rotz(-90))
					.difference(ServoHornCutout)
					.difference(squareNut)
					.difference(squareNut.rotz(-90))
			//return [stl, asm]
			//println "Cutting the bottom off the existing gear"
			CSG cutGear = stl.difference(stl.getBoundingBox().toZMax())
			//println "Adding lower Section Back On"
			cachedGearLink=cutGear.union(asm)
		}
		cachedGearLink.setColor(Color.RED)
		return cachedGearLink
	}
	CSG getGearLinkKeepaway() {
		CSG gl = getGearLink() ;
		CSG toCSGMovez = new Cylinder(gl.getTotalX()/2, gl.getTotalZ(),(int)40).toCSG().movez(gl.getMinZ())
		toCSGMovez.setIsWireFrame(true);
		return toCSGMovez
	}
	
	double getDistanceFromCenterToMotorTop(DHParameterKinematics d, int linkIndex) {
		// read motor typ information out of the link configuration
		LinkConfiguration conf = d.getLinkConfiguration(linkIndex);
		CSG motor = Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
		.toZMax()
		.movez(numbers.JointSpacing/2.0+numbers.LooseTolerance)
	// Is this value actually something in the CSV?
		return motor.getMaxZ()-numbers.LooseTolerance;
	}
	@Override
	public ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {
		if(d.getScriptingName().startsWith("Head")||d.getScriptingName().startsWith("Tail")) {
			return generateCadHeadTail(d, linkIndex)
		}
		AbstractLink link =d.getAbstractLink(linkIndex)
		// chaeck to see if this is the left side
		boolean left=false;
		boolean front=false;
		boolean isDummyGearWrist = false;
		double parametric = numbers.LinkLength-endOfPassiveLinkToBolt

		double l0offset =38.0-(numbers.LinkLength-endOfPassiveLinkToBolt)
		double l1offset = 55.0 -(numbers.LinkLength-endOfPassiveLinkToBolt)

		if(linkIndex==0) {
			parametric=d.getDH_R(linkIndex)-l0offset
		}
		if(linkIndex==1) {
			parametric=d.getDH_R(linkIndex)-l1offset
		}

		if(d.getScriptingName().startsWith("Dummy")) {
			isDummyGearWrist=true;
		}
		if(d.getRobotToFiducialTransform().getY()>0) {
			left=true;
		}
		if(d.getRobotToFiducialTransform().getX()>0) {
			front=true;
		}
		// read motor typ information out of the link configuration
		LinkConfiguration conf = d.getLinkConfiguration(linkIndex);
		// load the vitamin for the servo
		tailLength.setMM(0.1);
		CSG motor = Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
				.toZMax()
				.movez(numbers.JointSpacing/2.0+numbers.LooseTolerance)
		// Is this value actually something in the CSV?
		double distanceToMotorTop = getDistanceFromCenterToMotorTop(d,linkIndex);
		
		println "Center to horn distance "+distanceToMotorTop
		// a list of CSG objects to be rendered
		ArrayList<CSG> back =[]
		// get the UI manipulator for the link
		Affine dGetLinkObjectManipulator = d.getLinkObjectManipulator(linkIndex)
		// UI manipulator for the root of the limb
		Affine root = d.getRootListener()

		double link1Rotz=-90
		double MototRetractDist =15
		String motorKey = "Motor"+d.getScriptingName()+":"+linkIndex

		String leftMotorScrewKey = "LeftMotorScrew"+d.getScriptingName()+":"+linkIndex
		String rightMotorScrewKey = "RightMotorScrew"+d.getScriptingName()+":"+linkIndex
		String leftLinkScrewKey = "LeftLinkScrew"+d.getScriptingName()+":"+linkIndex
		String rightLinkScrewKey = "RightLinkScrew"+d.getScriptingName()+":"+linkIndex
		String leftLinkNutKey = "LeftLinkNut"+d.getScriptingName()+":"+linkIndex
		String rightLinkNutKey = "RightLinkNut"+d.getScriptingName()+":"+linkIndex
		String leftCalibrationNutKey = "LeftCalibrationNut"+d.getScriptingName()+":"+linkIndex
		String rightCalibrationNutKey = "RightCalibrationNut"+d.getScriptingName()+":"+linkIndex

		String leftCalibrationScrewKey = "LeftCalibrationScrew"+d.getScriptingName()+":"+linkIndex
		String rightCalibrationScrewKey = "RightCalibrationScrew"+d.getScriptingName()+":"+linkIndex

		String motorDoorScrewKey = "MotorDoorScrew"+d.getScriptingName()+":"+linkIndex
		String leftmotorDoorScrewKey = "LeftMotorDoorScrew"+d.getScriptingName()+":"+linkIndex
		String rightmotorDoorScrewKey = "RightMotorDoorScrew"+d.getScriptingName()+":"+linkIndex


		new VitaminLocation(false,motorKey,conf.getElectroMechanicalType(),conf.getElectroMechanicalSize(),new TransformNR(),link)

		new VitaminLocation(false,leftMotorScrewKey,"PhillipsRoundedHeadThreadFormingScrews","M2x8",new TransformNR(),link)
		new VitaminLocation(false,rightMotorScrewKey,"PhillipsRoundedHeadThreadFormingScrews","M2x8",new TransformNR(),link)



		new VitaminLocation(false,leftLinkNutKey,"squareNut","M3",new TransformNR(),link)
		new VitaminLocation(false,rightLinkNutKey,"squareNut","M3",new TransformNR(),link)

		new VitaminLocation(false,leftCalibrationNutKey,"squareNut","M3",new TransformNR(),link)
		new VitaminLocation(false,rightCalibrationNutKey,"squareNut","M3",new TransformNR(),link)


		new VitaminLocation(false,leftCalibrationScrewKey,"conePointSetScrew","M3x8",new TransformNR(),link)
		new VitaminLocation(false,rightCalibrationScrewKey,"conePointSetScrew","M3x8",new TransformNR(),link)
		LengthParameter facets		= new LengthParameter("Bolt Hole Facet Count",10,[40, 10])
		facets.setMM(30)
		offset.setMM(numbers.LooseTolerance)
		if(linkIndex==0) {

			new VitaminLocation(false,leftmotorDoorScrewKey,"PhillipsRoundedHeadThreadFormingScrews","M2x8",new TransformNR(),link)
			new VitaminLocation(false,rightmotorDoorScrewKey,"PhillipsRoundedHeadThreadFormingScrews","M2x8",new TransformNR(),link)

			motor=motor.rotz(left?180:0)
			motor=motor.roty(front?180:0)
			// the first link motor is located in the body
			motor.setManipulator(root)
			// pull the limb servos out the top
			motor.addAssemblyStep(4, new Transform().movex(isDummyGearWrist?-30:MototRetractDist))
			motor.addAssemblyStep(3, new Transform().movey(isDummyGearWrist?-30:left?-MototRetractDist*4:MototRetractDist*4))
		}else {
			new VitaminLocation(false,motorDoorScrewKey,"PhillipsRoundedHeadThreadFormingScrews","M2x8",new TransformNR(),link)
			motor=motor.roty(left?180:0)
			motor=motor.rotz(linkIndex==2?90:90+link1Rotz)
			if(linkIndex==1) {
				motor=motor.mirrory()
			}
			// the rest of the motors are located in the preior link's kinematic frame
			motor.setManipulator(d.getLinkObjectManipulator(linkIndex-1))
			// pull the link motors out the thin side
			motor.addAssemblyStep(6, new Transform().movey(linkIndex==1?MototRetractDist*2:0).movey(linkIndex==2?-MototRetractDist*2:0))

			motor.addAssemblyStep(7, new Transform().movex(linkIndex==1?MototRetractDist*2:0).movey(linkIndex==2?-MototRetractDist*2:0))
			//motor.addAssemblyStep(8, new Transform().movex(-30))
		}
		// do not export the motors to STL for manufacturing
		motor.setManufacturing({return null})
		motor.setColor(Color.BLUE)
		//Start the horn link
		// move the horn from tip of the link space, to the Motor of the last link space
		// note the hore is moved to the centerline distance value before the transform to link space

		CSG movedHorn = resinPrintServoMount.movez(distanceToMotorTop)
		if(linkIndex==0)
			movedHorn=movedHorn.roty(front?180:0)
		else
			movedHorn=movedHorn.roty(left?180:0)
		CSG myServoHorn = moveDHValues(movedHorn,d,linkIndex)
		if(!isDummyGearWrist) {
			if(linkIndex==0)
				myServoHorn.addAssemblyStep(9, new Transform().movey(front?10:-10))
			else
				myServoHorn.addAssemblyStep(9, new Transform().movez(left?-10:10))
		}else {
			myServoHorn.addAssemblyStep(4, new Transform().movex(isDummyGearWrist?-30:30))
		}
		//reorent the horn for resin printing
		myServoHorn.setManufacturing({incoming ->
			return null;//reverseDHValues(incoming, d, linkIndex).roty(linkIndex==0?(front?180:0):(left?180:0)).toZMin()
			//.roty(45)
			//.movez(5)
		})
		//myServoHorn.getStorage().set("bedType", "resin")
		//myServoHorn.setPrintBedNumber(4)
		//myServoHorn.setName("Resin Horn "+linkIndex+" "+d.getScriptingName())
		// attach this links manipulator
		myServoHorn.setManipulator(dGetLinkObjectManipulator)
		back.add(myServoHorn)
		//end horn link

		if(isDummyGearWrist) {
			motor.addAssemblyStep(3, new Transform().movez(front?60:-60))
			myServoHorn.addAssemblyStep(3, new Transform().movey(front?-50:50))

			CSG tmp = getGearLink()
					.roty(front?180:0)
			if(front)
				tmp=tmp.toZMax()
			else
				tmp=tmp.toZMin()
			distanceToMotorTop+=1.5
			tmp=tmp.movez((front?-1:1)*distanceToMotorTop)
			CSG wrist= moveDHValues(tmp, d, linkIndex)
			wrist.addAssemblyStep(4, new Transform().movex(isDummyGearWrist?-30:30))
			wrist.addAssemblyStep(3, new Transform().movey(front?-20:20))

			//.rotx(90)
			wrist.setName("DriveGear"+d.getScriptingName())
			wrist.setManufacturing({ incoming ->
				return incoming.rotx(front?-90:90).toZMin().toXMin().toYMin()
			})
			wrist.getStorage().set("bedType", "ff-One")
			wrist.setPrintBedNumber(1)
			wrist.setManipulator(d.getLinkObjectManipulator(linkIndex))
			back.add(wrist)
		}else {
			double coverDistance=80
			if(linkIndex==1) {
				// this section is a place holder to visualize the tip of the limb
				CSG kneeCover = Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/OperationSmallKat/Marcos.git",
						"ShoulderCover.stl"))
						.rotz(link1Rotz)
				if(left)
					kneeCover=kneeCover.mirrorz()
				kneeCover=kneeCover.mirrory()
				kneeCover.setManipulator(d.getLinkObjectManipulator(linkIndex-1))
				kneeCover.setManufacturing({incoming->
					return incoming.rotx(-90).toZMin().roty(90).toZMin()
				})
				kneeCover.getStorage().set("bedType", "ff-Two")
				kneeCover.setPrintBedNumber(left?2:5)
				kneeCover.setName("ShoulderCover"+d.getScriptingName())
				kneeCover.addAssemblyStep(12, new Transform().movex(10))
				kneeCover.addAssemblyStep(11, new Transform().movez(left?-coverDistance:coverDistance))
				back.add(kneeCover)

				CSG knee = Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/OperationSmallKat/Marcos.git",
						"Shoulder.stl"))
						.rotz(link1Rotz)
				if(left)
					knee=knee.mirrorz()
				knee=knee.mirrory()
				knee.setManipulator(d.getLinkObjectManipulator(linkIndex-1))
				knee.setManufacturing({incoming->
					return incoming.rotx(-90).roty(-90).toZMin()
				})
				knee.getStorage().set("bedType", "ff-Two")
				knee.setPrintBedNumber(2)
				knee.setName("Shoulder"+d.getScriptingName())
				back.add(knee)
			}
			if(linkIndex==2) {
				// this section is a place holder to visualize the tip of the limb
				CSG kneeCover = Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/OperationSmallKat/Marcos.git",
						"KneeCover.stl"))
						.rotx(180)
				if(!left)
					kneeCover=kneeCover.mirrorz()
				kneeCover.setManipulator(d.getLinkObjectManipulator(linkIndex-1))
				kneeCover.setManufacturing({incoming->
					return incoming.rotx(180).rotx(-90).toZMin()
				})
				kneeCover.getStorage().set("bedType", "ff-One")
				kneeCover.setPrintBedNumber(1)
				kneeCover.setName("KneeCover"+d.getScriptingName())
				kneeCover.addAssemblyStep(12, new Transform().movey(-10))
				kneeCover.addAssemblyStep(11, new Transform().movez(left?-coverDistance:coverDistance))
				back.add(kneeCover)

				CSG knee = Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/OperationSmallKat/Marcos.git",
						"Knee"+(left?"Left":"Right")+".stl"))
						.rotx(180)
				knee.setManipulator(d.getLinkObjectManipulator(linkIndex-1))
				knee.setManufacturing({incoming->
					return incoming.rotx(-180).rotx(-90).toZMin().rotz(left?180:0)
				})
				knee.getStorage().set("bedType", "ff-One")
				knee.setPrintBedNumber(1)
				knee.setName("Knee"+d.getScriptingName())
				back.add(knee)

				CSG foot = getFoot(d.getDH_R(linkIndex))
				foot.setManipulator(dGetLinkObjectManipulator)
				foot.setManufacturing({incoming->
					return incoming.rotx(90).roty(90-numbers.FootAngle).toZMin().rotz(front?180:0)
				})
				foot.getStorage().set("bedType", "ff-Two")
				foot.setPrintBedNumber(3)
				foot.setName("Foot"+d.getScriptingName())
				back.add(foot)
			}
			double zrotValDrive = -d.getDH_Theta(linkIndex)
			if(linkIndex==1) {
				zrotValDrive+=45
			}
			if(linkIndex==2) {
				zrotValDrive+=(-90+numbers.FootAngle)+1
			}
			double zrotVal = -d.getDH_Theta(linkIndex)
			if(linkIndex==1) {
				zrotVal+=45
			}
			if(linkIndex==2) {
				zrotVal+=(-90+numbers.FootAngle)
			}


			VitaminLocation vLLS=new VitaminLocation(false,leftLinkScrewKey,"chamferedScrew","M3x16",new TransformNR().translateX(parametric),link)
			VitaminLocation vRLS=new VitaminLocation(false,rightLinkScrewKey,"chamferedScrew","M3x16",new TransformNR().translateX(parametric),link)
			CSG boltlStart = MobileBaseCadManager.get(link).getVitamin(vLLS)
			CSG boltrStart = MobileBaseCadManager.get(link).getVitamin(vRLS)


			CSG boltl= moveDHValues(boltlStart
					.rotx(180)
					.toZMin()
					.movez(-(distanceToMotorTop+numbers.LinkHeight))
					.rotz(zrotVal)
					,d,linkIndex)
			CSG boltr= moveDHValues(boltrStart
					.rotx(0)
					.toZMax()
					.movez((distanceToMotorTop+numbers.LinkHeight))
					.rotz(zrotVal)
					,d,linkIndex)

			boltl.setManipulator(dGetLinkObjectManipulator)
			boltr.setManipulator(dGetLinkObjectManipulator)
			boltl.setManufacturing({return null})
			boltr.setManufacturing({return null})

			CSG movedDrive = calibrationLink(parametric,boltlStart)
					.movez(distanceToMotorTop)//.rotz(180)
			double xrotDrive=180
			xrotDrive+=linkIndex==0&&(!front)?180:0
			xrotDrive+=linkIndex!=0&&(!left)?180:0
			movedDrive=movedDrive.rotx(xrotDrive)

			movedDrive=movedDrive.rotz(zrotValDrive)
			CSG myDriveLink = moveDHValues(movedDrive,d,linkIndex)
			back.addAll([boltr, boltl])

			if(linkIndex==0) {
				boltl.addAssemblyStep(10, new Transform().movey(25))
				myDriveLink.addAssemblyStep(9, new Transform().movey(front?20:-20))
			}else {
				boltl.addAssemblyStep(10, new Transform().movez(-25))
				myDriveLink.addAssemblyStep(9, new Transform().movez(left?-20:20))
			}

			//reorent the horn for resin printing
			myDriveLink.setManufacturing({incoming ->
				return reverseDHValues(incoming.rotz(-zrotValDrive).rotx(-xrotDrive), d, linkIndex).toZMin()
			})
			myDriveLink.getStorage().set("bedType", "ff-Two")
			myDriveLink.setPrintBedNumber(2)
			myDriveLink.setName("DriveLink "+linkIndex+" "+d.getScriptingName())
			// attach this links manipulator
			myDriveLink.setManipulator(dGetLinkObjectManipulator)
			back.add(myDriveLink)

			double kinematicsLen = d.getDH_R(linkIndex)
			double staticOffset = 55.500-numbers.LinkLength-endOfPassiveLinkToBolt
			double calculated = kinematicsLen-staticOffset

			double xrot=0
			CSG linkCSG = passiveLink(parametric,boltrStart)
					.movez(distanceToMotorTop)
			xrot+=linkIndex==0&&(!front)?180:0
			xrot+=linkIndex!=0&&(!left)?180:0
			linkCSG=linkCSG.rotx(xrot)

			linkCSG=linkCSG.rotz(zrotVal)
			CSG wrist= moveDHValues(linkCSG, d, linkIndex)
			if(linkIndex!=0) {
				double dist = 25;
				boltr.addAssemblyStep(10, new Transform().movez(dist))
				wrist.addAssemblyStep(10, new Transform().movez(left?5:-5))
			}else {
				boltr.addAssemblyStep(10, new Transform().movey(-25))
				wrist.addAssemblyStep(10, new Transform().movey(front?-5:5))
			}
			//.rotx(90)
			wrist.setName("PassiveLink"+d.getScriptingName()+linkIndex)
			wrist.setManufacturing({ incoming ->
				return reverseDHValues( incoming.rotz(-zrotVal).rotx(-xrot), d, linkIndex).toZMin().toXMin().toYMin()
			})
			wrist.getStorage().set("bedType", "ff-Two")
			wrist.setPrintBedNumber(2)
			wrist.setManipulator(d.getLinkObjectManipulator(linkIndex))
			back.add(wrist)
		}
		motor.setName(conf.getElectroMechanicalSize())
		back.add(motor)
		cache.addAll(back)
		return back;
	}
	CSG getNeckLink(AbstractLink link) {
		double neckLenFudge = 4.5
		double parametric = numbers.LinkLength-endOfPassiveLinkToBolt
		String rightLinkScrewKey="RightLinkScrewTail:1"
		double length =parametric+neckLenFudge-0.75
		
		VitaminLocation v= new VitaminLocation(false,rightLinkScrewKey,"chamferedScrew","M3x16",new TransformNR().translateX(length),link)
		CSG boltl = MobileBaseCadManager.get(link).getVitamin(v)
		return passiveLink(length,boltl)
				.rotx(180)
				.movez(-15.1)
	}
	public ArrayList<CSG> generateCadHeadTail(DHParameterKinematics d, int linkIndex) {
		boolean left=false;
		boolean front=false;
		boolean isDummyGearWrist = false;

		String leftLinkScrewKey = "LeftLinkScrew"+d.getScriptingName()+":"+linkIndex
		String rightLinkScrewKey = "RightLinkScrew"+d.getScriptingName()+":"+linkIndex
		String leftLinkNutKey = "LeftLinkNut"+d.getScriptingName()+":"+linkIndex
		String rightLinkNutKey = "RightLinkNut"+d.getScriptingName()+":"+linkIndex

		String MountScrewKey = "MountScrew"+d.getScriptingName()+":"+linkIndex
		String MountNutKey = "MountNut"+d.getScriptingName()+":"+linkIndex

		if(d.getScriptingName().startsWith("Dummy")) {
			isDummyGearWrist=true;
		}
		if(d.getRobotToFiducialTransform().getY()>0) {
			left=true;
		}
		if(d.getRobotToFiducialTransform().getX()>0) {
			front=true;
		}
		ArrayList<CSG> back =[]
		double wristCenterOffset = 6
		if(linkIndex==0) {
			CSG wrist= Vitamins.get(ScriptingEngine.fileFromGit(
					"https://github.com/OperationSmallKat/Marcos.git",
					"WristCenter.stl"))
					.rotz(90)
					.movez(-wristCenterOffset)
			wrist.addAssemblyStep(4, new Transform().movez(30))

			wrist.setName("WristCenter"+d.getScriptingName())
			wrist.setManufacturing({ incoming ->
				return incoming.roty(90).toZMin().toXMin().toYMin()
			})
			wrist.getStorage().set("bedType", "ff-One")
			wrist.setPrintBedNumber(1)
			back.add(wrist)
			new VitaminLocation(false,MountScrewKey,"capScrew","M3x16",new TransformNR())
			new VitaminLocation(false,MountNutKey,"squareNut","M3",new TransformNR())
		}
		if(linkIndex==1) {
			new VitaminLocation(false,leftLinkScrewKey,"chamferedScrew","M3x16",new TransformNR())
			new VitaminLocation(false,leftLinkNutKey,"squareNut","M3",new TransformNR())
			new VitaminLocation(false,rightLinkNutKey,"squareNut","M3",new TransformNR())

			String name= d.getScriptingName();
			CSG link = getNeckLink(d.getAbstractLink(1))

			link.addAssemblyStep(4, new Transform().movez(30))
			link.addAssemblyStep(2, new Transform().movez(-30))

			link.setName("PassiveLink"+d.getScriptingName())
			link.setManufacturing({ incoming ->
				return incoming.roty(180).toZMin().toXMin().toYMin()
			})
			link.getStorage().set("bedType", "ff-Three")
			link.setPrintBedNumber(2)
			back.add(link)
			CSG gearLink= Vitamins.get(ScriptingEngine.fileFromGit(
					"https://github.com/OperationSmallKat/Marcos.git",
					"GearLinkChamfer.stl"))

					.movez(16.25)
			gearLink.addAssemblyStep(4, new Transform().movez(30))
			gearLink.addAssemblyStep(2, new Transform().movez(30))

			gearLink.setName("GearLink"+d.getScriptingName())
			gearLink.setManufacturing({ incoming ->
				return incoming.roty(180).toZMin().toXMin().toYMin()
			})
			gearLink.getStorage().set("bedType", "ff-One")
			gearLink.setPrintBedNumber(1)
			back.add(gearLink)

			CSG headtail= Vitamins.get(ScriptingEngine.fileFromGit(
					"https://github.com/OperationSmallKat/Marcos.git",
					name+".stl"))
					.rotz(180)
					.toXMin()
			if(name.contentEquals("Head")) {
				headtail=headtail.roty(-37)
						.movez(-wristCenterOffset-1)
						.movex(6.6)
				headtail.setManufacturing({ incoming ->
					return incoming.roty(90).toZMin().toXMin().toYMin()
				})
			}
			if(name.contentEquals("Tail")) {
				headtail=headtail
						.movez(-0.7)
						.movex(26.7)
						.movey(-headtail.getTotalY()/2)
				headtail.setManufacturing({ incoming ->
					return incoming.rotx(90).toZMin().toXMin().toYMin()
				})
			}
			headtail.addAssemblyStep(4, new Transform().movez(30))
			headtail.addAssemblyStep(2, new Transform().movex(30))

			headtail.setName(name+"_"+d.getScriptingName())

			headtail.getStorage().set("bedType", "ff-One")
			headtail.setPrintBedNumber(1)
			back.add(headtail)
		}

		for(CSG c:back)
			c.setManipulator(d.getLinkObjectManipulator(linkIndex))
		cache.addAll(back)
		return back;
	}
	CSG getLinkMountBLock() {
		double linkWidth = numbers.LinkWidth
		double linkThickness = numbers.LinkHeight
		double z = numbers.JointSpacing+linkThickness*2
		double x = 13
		double y =numbers.LinkWidth
		double smallChamfer = numbers.Chamfer1/1.5

		double linklen=numbers.LinkLength-endOfPassiveLinkToBolt
		double SquareNutWidth = numbers.SquareNutWidth + numbers.LooseTolerance
		double SquareNutHeight = numbers.SquareNutHeight + numbers.LooseTolerance
		double SquareNutCutOutHeight = linkThickness/2+SquareNutWidth/2
		double LinkSqaureNutSpacing = numbers.LinkSqaureNutSpacing
		double linkRadius = linkWidth/2
		CSG SquareNutChamfer = StraightChamfer(SquareNutHeight,SquareNutWidth,smallChamfer)

		CSG SquareNutCutOut = new Cube(SquareNutHeight,SquareNutWidth, SquareNutWidth/2+x/2).toCSG()
				.toZMin()
				.union(SquareNutChamfer)
				.roty(-90)
				.movez(z/2-numbers.MountingScrewLength+SquareNutHeight)
				.movex(-x/2)




		CSG cutout = linkCore(linklen,null,0)
				.movex(-linklen)
				.movez(numbers.JointSpacing/2)
		CSG boltHole = new Cylinder((numbers.MountingScrewDiamter+numbers.HoleTolerance)/2.0, z).toCSG()
				.movez(-z/2.0)
		CSG block = StraightChamfer(x,y,z,numbers.Chamfer3,numbers.Chamfer2)
				.difference(boltHole)
				.difference(SquareNutCutOut)
				.difference(SquareNutCutOut.mirrorz())
				.difference(cutout)
				.difference(cutout.mirrorz())



		return block
	}
	CSG getFoot(double linkLen) {
		double angle = numbers.FootAngle
		double linkWidth = numbers.FootWidth

		double linklen=numbers.LinkLength-endOfPassiveLinkToBolt
		CSG cutout = linkCore(linklen,null,0)
				.rotz(angle)
				.movez(numbers.JointSpacing/2)
				.movex(-linkLen)
		CSG linkMountBlock = getLinkMountBLock()
				.movex(linklen)
				.rotz(angle)
				.movex(-linkLen)

//		CSG foot  = Vitamins.get(ScriptingEngine.fileFromGit(
//				"https://github.com/OperationSmallKat/Marcos.git",
//				"Foot.stl"))
//				.rotx(180)
		double c3 = Math.sin(Math.toRadians(angle))*linklen
		double c1 = Math.cos(Math.toRadians(angle))*linklen
		double c2= linkLen-c1;
		double hyp = Math.sqrt(Math.pow(c3, 2)+Math.pow(c2,2))-(numbers.MountingScrewDiamter+numbers.HoleTolerance)*2+numbers.Chamfer3
		double linkangel = Math.toDegrees(Math.atan2(c3, c2))


		CSG ball = footBallSection()
				.rotx(-90)
				.rotz(-angle)
		double linkXOffset = ball.getMinX()+numbers.Chamfer3*2
		double recourveDepth =5

		double recurrveRad = hyp*1.5

		CSG recurveChamfer = ChamferedCylinderHR(recurrveRad+numbers.Chamfer3,numbers.Chamfer3*3, numbers.Chamfer3)
		CSG recurve = new Cylinder(recurrveRad,recurrveRad, linkWidth,80).toCSG()
				.union(recurveChamfer.toZMax().movez(numbers.Chamfer3))
				.union(recurveChamfer.movez(linkWidth-numbers.Chamfer3))
				.toYMax()
				.movey(-linkWidth/2 + recourveDepth)
				.movez(-linkWidth/2)
		CSG linkPart = new ChamferedCube(hyp+linkXOffset,linkWidth,linkWidth,numbers.Chamfer3).toCSG()
				.difference(recurve)
				.toXMax()
				.movex(linkXOffset)
				.rotz(-linkangel)
		CSG pawCap =new ChamferedCube(linkWidth+1,linkWidth-0.5,linkWidth,numbers.Chamfer3).toCSG()
				.toXMax()
				.movex(2)
				.toYMin()
				.movey(-0.5)
				.rotz(-angle)
		return CSG.unionAll([
			ball,
			linkMountBlock,
			linkPart,
			pawCap
		]).setColor(Color.DARKRED)
	}
	DHParameterKinematics getByName(MobileBase b,String name) {
		for(DHParameterKinematics k:b.getAllDHChains()) {
			if(k.getScriptingName().contentEquals(name))
				return k
		}
		return null;
	}
	public void setMobileBase(MobileBase mb) {
		this.mb = mb;
		
		
//		if(bom==null)
//			bom=new VitaminBomManager(mb.getGitSelfSource()[0]);
	}
	TransformNR transformToTipOfLink(DHParameterKinematics d, int linkIndex){
		TransformNR linkTip;
		try {
			linkTip = d.getLinkTip(linkIndex);
			if(linkTip==null)
				throw new RuntimeException();
		}catch(Exception e) {
			linkTip=d.getChain().getChain(d.getCurrentJointSpaceVector()).get(linkIndex);
		}
		return linkTip
	}
	@Override
	public ArrayList<CSG> generateBody(MobileBase base) {
		boolean makeCalibration =true;
		cache.clear()
		setMobileBase(base)
		DHParameterKinematics dh = base.getLegs().get(0);
		double zCenterLine = dh.getRobotToFiducialTransform().getZ()+numbers.ServoThickness/2.0;

		TransformNR batteryLocation =new TransformNR()
				.translateZ(zCenterLine)
		TransformNR motherboardLocation =batteryLocation.copy()
				.translateZ(13.5)
				.translateX(-1.5)
		batteryLocation.setRotation(RotationNR.getRotationX(180))
		batteryLocation=batteryLocation.times(new TransformNR(0,0,0,RotationNR.getRotationZ(90)))
		TransformNR batteryInterfaceLocation =batteryLocation.copy()
				.translateX(49.25)
				.translateZ(-12.5)

		batteryLocation.translateZ(7)
		batteryLocation.translateX(-3)
		
		new VitaminLocation(false,"MotherboardScrew1","PhillipsRoundedHeadThreadFormingScrews","M3x6",new TransformNR(),base)
		new VitaminLocation(false,"MotherboardScrew2","PhillipsRoundedHeadThreadFormingScrews","M3x6",new TransformNR(),base)
		new VitaminLocation(false,"MotherboardScrew3","PhillipsRoundedHeadThreadFormingScrews","M3x6",new TransformNR(),base)
		new VitaminLocation(false,"MotherboardScrew4","PhillipsRoundedHeadThreadFormingScrews","M3x6",new TransformNR(),base)
		new VitaminLocation(false,"BatteryScrew5","PhillipsRoundedHeadThreadFormingScrews","M3x6",new TransformNR(),base)
		new VitaminLocation(false,"BatteryScrew6","PhillipsRoundedHeadThreadFormingScrews","M3x6",new TransformNR(),base)

		VitaminLocation battv = new VitaminLocation(false,"battery","smallKatElectronics","dji-mavic-pro-battery",batteryLocation,base)
		VitaminLocation intf= new VitaminLocation(false,"batteryInterface","smallKatElectronics","batteryInterface",batteryInterfaceLocation,base)
		VitaminLocation mobo = new VitaminLocation(false,"motherboard","smallKatElectronics","motherboard",motherboardLocation,base)


		new VitaminLocation(false,"CoverScrew1","chamferedScrew","M3x16",new TransformNR(),base)
		new VitaminLocation(false,"CoverScrew2","chamferedScrew","M3x16",new TransformNR(),base)
		new VitaminLocation(false,"CoverScrew3","chamferedScrew","M3x16",new TransformNR(),base)
		new VitaminLocation(false,"CoverScrew4","chamferedScrew","M3x16",new TransformNR(),base)



		CSG battery = MobileBaseCadManager.get(base).getVitamin(battv)
				.addAssemblyStep(13, new Transform().movez(-40))
		CSG batteryInterface =  MobileBaseCadManager.get(base).getVitamin(intf)
				.addAssemblyStep(5, new Transform().movez(-40))
		CSG motherboard = MobileBaseCadManager.get(base).getVitamin(mobo)
				.addAssemblyStep(5, new Transform().movez(40))


		CSG body  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"BodyRib.stl")).movez(zCenterLine);
		CSG bodyCOver  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"CoverHinged.stl")).movez(zCenterLine);
		CSG bodyLatch  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"Latch.stl")).movez(zCenterLine);
		CSG topCOver  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"BodyServoCoverTop.stl")).movez(zCenterLine);
		CSG BottomCover  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"BodyCoverBottom.stl")).movez(zCenterLine);
		ArrayList<CSG> back =[body, bodyCOver,bodyLatch]
		for(CSG c:back) {
			c.getStorage().set("bedType", "ff-One")
			c.setPrintBedNumber(5)
		}
		for(DHParameterKinematics k:base.getLegs()) {
			boolean left=false;
			boolean front=false;
			if(k.getRobotToFiducialTransform().getY()>0) {
				left=true;
			}
			if(k.getRobotToFiducialTransform().getX()>0) {
				front=true;
			}
			CSG top = topCOver;
			CSG bottom = BottomCover
			if(!left) {
				top=top.mirrorx()
				bottom=bottom.mirrorx()
			}
			if(!front) {
				top=top.mirrory()
				bottom=bottom.mirrory()
			}
			def local = "ServoCoverTop"+(left?"Left":"Right")+(front?"Front":"Back")
			top.setName(local);
			def local2 = "ServoCoverBottom"+(left?"Left":"Right")+(front?"Front":"Back")
			bottom.setName(local2);
			top.setManufacturing({ incoming ->
				return incoming.toZMin().toXMin().toYMin()
			})
			bottom.setManufacturing({ incoming ->
				return incoming.toZMin().toXMin().toYMin().movey(top.getTotalY()+1)
			})
			top.getStorage().set("bedType", "ff-Two")
			top.setPrintBedNumber(3)
			bottom.getStorage().set("bedType", "ff-Two")
			bottom.setPrintBedNumber(3)
			top.addAssemblyStep(6, new Transform().movez(10))
			bottom.addAssemblyStep(6, new Transform().movez(-10))
			double distacne = front?80:-80
			top.addAssemblyStep(5, new Transform().movey(distacne))
			bottom.addAssemblyStep(5, new Transform().movey(distacne))

			back.addAll([top, bottom])
			println "ServoCover's for "+(left?"Left":"Right")+(front?"Front":"Back")
		}
		// Set the location of the limbs based on the CSV in the body loader
		for(DHParameterKinematics d:base.getAllDHChains()) {
			boolean left=false;
			boolean front=false;
			boolean isDummyGearWrist = false;
			if(d.getScriptingName().startsWith("Dummy")) {
				isDummyGearWrist=true;
			}
			if(d.getRobotToFiducialTransform().getY()>0) {
				left=true;
			}
			if(d.getRobotToFiducialTransform().getX()>0) {
				front=true;
			}
			TransformNR dGetRobotToFiducialTransform = d.getRobotToFiducialTransform()
			def legTOSHoulderX = 7.5
			def legTOSHoulderY = 6.5

			double xval=(numbers.BodyServoCenterLength/2.0+legTOSHoulderX)*(front?1.0:-1.0)
			if(!isDummyGearWrist) {
				if(base.getLegs().contains(d)) {
					dGetRobotToFiducialTransform.setY(numbers.BodyServoCenterWidth/2.0*(left?1.0:-1.0))
					dGetRobotToFiducialTransform.setX(numbers.BodyServoCenterLength/2.0*(front?1.0:-1.0))
				}
				if(d.getScriptingName().startsWith("Head")||d.getScriptingName().startsWith("Tail")) {
					dGetRobotToFiducialTransform.setX(xval)
				}
			}else {
				dGetRobotToFiducialTransform.setY((numbers.BodyServoCenterWidth/2.0-legTOSHoulderY)*(left?1.0:-1.0))
				dGetRobotToFiducialTransform.setX(xval)
			}
			d.setRobotToFiducialTransform(dGetRobotToFiducialTransform)
		}

		bodyCOver.setName("BodyCover")
		bodyCOver.addAssemblyStep(6, new Transform().movez(80))
		body.setName("Body")
		bodyLatch.setName("Latch")
		bodyLatch.setManufacturing({ incoming ->
			return incoming.toZMin().toXMin().toYMin()
		})
		body.setManufacturing({ incoming ->
			return incoming.rotx(180).toZMin().toXMin().toYMin()
		})
		bodyCOver.setManufacturing({ incoming ->
			return incoming.toZMin().toXMin().toYMin().movey(body.getTotalY()+1)
		})
		File workDir = ScriptingEngine.getRepositoryCloneDirectory(base.getGitSelfSource()[0]);
		GitVersionCalculator jgitver = GitVersionCalculator.location(workDir).setMavenLike(true)

		String semver = jgitver.getVersion()
		String configHash = base.getXml().hashCode()+"-"+semver;
		Font font = new Font("Arial",  6);

		File calibrationJigFile = new File(workDir.getAbsolutePath()+"/CalibrationJig-"+configHash+".stl")
		System.out.println("Stand file "+calibrationJigFile.getAbsolutePath());
		CSG spars
		if(calibrationJigFile.exists()) {
			println "Calibration Jig Exists "+calibrationJigFile.getAbsolutePath()
			//makeCalibration=false
			spars  = Vitamins.get(calibrationJigFile);
		}
		if(makeCalibration) {
			double blockDepth = 25
			double blockHeight = 20
			double scoochUpDistance = 1
			CSG label = CSG.unionAll(TextExtrude.text((double)1.0,semver,font))
					.roty(180)
					.movex(-blockDepth)
					.toZMin()
					.moveToCenterY()


			Transform tipLeftFront = TransformFactory.nrToCSG(getByName(base,"LeftFront").calcHome())
			Transform tipRightFront = TransformFactory.nrToCSG(getByName(base,"RightFront").calcHome())
			Transform tipLeftRear = TransformFactory.nrToCSG(getByName(base,"LeftRear").calcHome())
			Transform tipRightRear = TransformFactory.nrToCSG(getByName(base,"RightRear").calcHome())

			DHParameterKinematics h = getByName(base,"Head")
			DHParameterKinematics t = getByName(base,"Tail")

			Transform neck =TransformFactory.nrToCSG(transformToTipOfLink(h,1))
			Transform butt =TransformFactory.nrToCSG(transformToTipOfLink(t,1))

			CSG neckBit = getNeckLink(h.getAbstractLink(1)).transformed(neck)
			CSG buttBit = getNeckLink(t.getAbstractLink(1)).transformed(butt)
			
			CSG calBlock = new ChamferedCube(blockDepth,25,blockHeight,numbers.Chamfer2).toCSG()
					.toZMin()
					.movez(scoochUpDistance)
			
			//.movez(5)
			CSG footLeftFront=getFoot(getByName(base,"LeftFront").getDH_R(2)).transformed(tipLeftFront)
			CSG footRightFront=getFoot(getByName(base,"RightFront").getDH_R(2)).transformed(tipRightFront)
			CSG footLeftRear=getFoot(getByName(base,"LeftRear").getDH_R(2)).transformed(tipLeftRear)
			CSG footRightRear=getFoot(getByName(base,"RightRear").getDH_R(2)).transformed(tipRightRear)

			CSG fCenter=calBlock.toXMax().move(tipLeftFront.x, 0, tipLeftFront.z)

			CSG rCenter=calBlock.move(tipRightRear.x, 0, tipRightRear.z)

			double calSinkInDistance =4
			CSG fCal = calBlock.union(calBlock.movex(5)).toZMax().move(neck.x, neck.y, neckBit.getMinZ()+numbers.Chamfer2+calSinkInDistance)
					.union(fCenter)
					.hull()
					.difference(neckBit)
			CSG rCal = calBlock.toZMax().move(butt.x, butt.y, buttBit.getMinZ()+numbers.Chamfer2+calSinkInDistance)
					.union(rCenter)
					.hull()
					.difference(buttBit)
			double inset=2
			CSG lowerBlock =  new ChamferedCube(10,25,blockHeight/2,numbers.Chamfer2).toCSG()
			.toZMax()
			.toXMax()
			.movex(-15)
			.movez(scoochUpDistance+calBlock.getMinZ()+3)
			CSG scoochedUp = calBlock
					.union(lowerBlock)
					.toXMax()
					.movex(12)
			CSG fl = scoochedUp.toYMax().movey(inset)
			CSG fr = scoochedUp.toYMin().movey(-inset)
			CSG rl = scoochedUp.toYMax().movey(inset)
			CSG rr = scoochedUp.toYMin().movey(-inset)
			CSG FrontSpar = fl.move(tipLeftFront.x, tipLeftFront.y, tipLeftFront.z)
					.union(fr.move(tipRightFront.x, tipRightFront.y, tipRightFront.z))
					.hull()
					.difference(footLeftFront)
					.difference(footRightFront)
			CSG RearSpar = rl.move(tipLeftRear.x, tipLeftRear.y, tipLeftRear.z)
					.union(rr.move(tipRightRear.x, tipRightRear.y, tipRightRear.z))
					.hull()
					.difference(footLeftRear)
					.difference(footRightRear)
			CSG Center = fCenter
					.union(rCenter)
					.hull()
					.toZMin()
					.movez(RearSpar.getMinZ())
					.movex(-15)
			label=label.movex(tipLeftFront.x-5)
					.movez(Center.getMaxZ())
			CSG gearAllignment = null;
			def gearLimbs = [getByName(base,"DummyRightFront"),
				getByName(base,"DummyLeftFront"),
				getByName(base,"DummyRightRear"),
				getByName(base,"DummyLeftRear")
				]
			CSG gearCutout = getGearLinkKeepaway()
			CSG gearRest = new  ChamferedCube(gearCutout.getTotalX(),gearCutout.getTotalY(),gearCutout.getTotalZ(),
				numbers.Chamfer2).toCSG()
								.toZMin()
								.movez(gearCutout.getMinZ())

			for(DHParameterKinematics k:gearLimbs) {
				Transform location =TransformFactory.nrToCSG(k.getRobotToFiducialTransform())
				boolean front = k.getScriptingName().toLowerCase().contains("front")
				double zdistance = (front?-1:1)*(getDistanceFromCenterToMotorTop(k, 0)+((front?0:1)*2.5))+
								(front?gearCutout.getMinZ():gearCutout.getMaxZ())
				location = location.apply(new Transform().movez(zdistance))
				CSG support =new Wedge(gearCutout.getTotalX()/2-(numbers.Chamfer2*2),
					gearCutout.getTotalY()-(numbers.Chamfer2*2),
					gearCutout.getTotalZ()-(numbers.Chamfer2*2)
					).toCSG()
					.rotx(front?0:180)
					.toZMin()
					.toYMin()
					.movey(gearRest.getMinY()+numbers.Chamfer2)
					.toXMin()
					.movex(gearRest.getMaxX())
					.movez(gearRest.getMinZ()+numbers.Chamfer2)
				CSG calibrationBlock = gearRest.union(support)
										.movez((front?-1:1)*-1.5)
										.movex(gearCutout.getTotalX()*0.75)
										.difference(gearCutout)
				CSG moved = calibrationBlock
							.transformed(location)
				if(gearAllignment==null)
					gearAllignment=moved
				else
					gearAllignment=gearAllignment.union(moved)
			}
			spars = Center.union([
				FrontSpar,
				RearSpar,
				fCal,
				rCal,
				label,
				gearAllignment
			])
			//		CSG LeftFrontbox=calBlock.move(tipLeftFront.x, tipLeftFront.y, tipLeftFront.z).difference(footLeftFront)
			//		CSG RightFrontbox=calBlock.move(tipRightFront.x, tipRightFront.y, tipRightFront.z).difference(footRightFront)
			//		CSG LeftRearbox=calBlock.move(tipLeftRear.x, tipLeftRear.y, tipLeftRear.z).difference(footLeftRear)
			//		CSG RightRearbox=calBlock.move(tipRightRear.x, tipRightRear.y, tipRightRear.z).difference(footRightRear)
			FileUtil.write(Paths.get(calibrationJigFile.getAbsolutePath()),
					spars.toStlString());
		}

			
		spars.setManufacturing({incoming -> return incoming.rotz(90).toZMin()})
		spars.setName("CalibrationJig")
		spars.getStorage().set("bedType", "ff-Three")
		spars.setPrintBedNumber(3)
		spars.setColor(Color.DARKRED)
		back.add(spars)
		spars.getStorage().set("no-physics", true);
		motherboard.getStorage().set("no-physics", true);
		back.addAll([
			battery,
			batteryInterface,
			motherboard
		])
		for(CSG c:back) {
			c.setManipulator(base.getRootListener())
		}
		//		for(DHParameterKinematics kin:arg0.getAllDHChains()) {
		//			CSG limbRoot =new Cube(1).toCSG()
		//			limbRoot.setManipulator(kin.getRootListener())
		//			back.add(limbRoot)
		//		}
		cache.addAll(back)
		return back;
	}
	public CSG foot() {
		double defaultValue = numbers.LinkLength - endOfPassiveLinkToBolt
		CSG stl= Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"DriveLink.stl"))

		double tolerance = numbers.Tolerance
		double chamfer = numbers.Chamfer2
		double largeChamfer = numbers.Chamfer3
		double smallChamfer = numbers.Chamfer1
		double linkWidth = numbers.LinkWidth
		double linkThickness = numbers.LinkHeight
		double linkRadius = linkWidth/2
		double filletRad=numbers.Fillet3

		double FootLength = numbers.FootLength
		double FootBaseWidth = numbers.FootBaseWidth
		double FootBaseHeight = numbers.FootBaseHeight
		double FootAngle = numbers.FootAngle
		double FootDiameter = numbers.FootDiameter
		double FootPawDiameter = numbers.FootPawDiameter
		double FootPawHeight1 = numbers.FootPawHeight1
		double FootPawHeight2 = numbers.FootPawHeight2
		double FootPawInnerDiameter = 13.4
		double FootPawAngle = numbers.FootPawAngle
		double FootPawRadius = numbers.FootPawRadius
		double FootArch = numbers.FootArch
		double FootWidth = numbers.FootWidth
		double JointSpacing = numbers.JointSpacing
		double LinkMountingCutOutWidth=numbers.LinkMountingCutOutWidth
		double LinkMountingCutOutLength=numbers.LinkMountingCutOutLength

		double ServoHornRad=(numbers.ServoHornDiameter+numbers.ServoHornHoleTolerance)/2.0
		double ServoHornHeight =numbers.ServoHornHeight+numbers.LooseTolerance
		double mountHeadRad =( numbers.MountingScrewHeadDiamter+numbers.LooseTolerance)/2.0
		double mountRad=(numbers.MountingScrewDiamter+numbers.HoleTolerance)/2.0
		double decritiveRad = numbers.ServoHornDiameter/4.0
		double SetscrewLength = numbers.SetScrewLength
		double SetscrewSize = numbers.SetScrewSize
		double SquareNutWidth = numbers.SquareNutWidth + numbers.LooseTolerance
		double SquareNutHeight = numbers.SquareNutHeight + numbers.LooseTolerance
		double SquareNutCutOutHeight = linkThickness/2+SquareNutWidth/2
		double LinkSqaureNutSpacing = numbers.LinkSqaureNutSpacing




		//Solving for Angle of setscrew.
		double hypot1 = Math.hypot(ServoHornRad + SetscrewLength + numbers.LooseTolerance, SetscrewSize/2)
		double angle1 = Math.asin(linkRadius/hypot1)
		double angle2 = Math.asin((ServoHornRad + SetscrewLength + numbers.LooseTolerance)/hypot1)
		double angle3 = (Math.PI/2)-angle1
		double angle4 = (Math.PI/2)-angle2
		double SetScrewAngle = Math.toDegrees((Math.PI/2)-(angle3+angle4))
		double SetScrewChamferLength = linkRadius/Math.sin((Math.PI/2)-(angle3+angle4))
		double SetScrewCutOutLength = numbers.LinkLength/Math.cos((Math.PI/2)-(angle3+angle4))

		// Hull together a toolshape to make the cutter to make the shape appropratly
		CSG ball = new Sphere(FootDiameter/2,40,40).toCSG()
		// cut from the corner to the ege of the link
		CSG spherecutter = new Cylinder((FootDiameter+1)/2, FootDiameter, 40).toCSG().toZMax()
		CSG pawbase = new Cylinder((FootPawInnerDiameter)/2, FootPawHeight2-FootPawHeight1, 40).toCSG().toZMax()
		CSG pawradius = new Sphere(FootPawRadius,40,40).toCSG().movez(-FootPawHeight2+FootPawHeight1).movex(FootPawRadius)
		pawradius = pawradius.difference(spherecutter.toZMin().movez(-FootPawHeight2+FootPawHeight1)).difference(spherecutter.movez(-FootPawHeight2))
		for(int i=0;i<=180;i++) {
			pawradius = pawradius.hull(pawradius.rotz(i*2))
		}

		//CSG base = CSG.unionAll(Extrude.revolve(pawradius, 0, 40))
		CSG paw = ball.difference(spherecutter).union(pawbase).union(pawradius)

		CSG LinkMountBlank = new ChamferedCube(linkWidth,(JointSpacing+linkThickness*2),(LinkMountingCutOutLength+tolerance+FootBaseWidth*2+chamfer*2),chamfer).toCSG()
		CSG LinkMountSideChamfer = new Cube(linkWidth,(JointSpacing+linkThickness*2),(LinkMountingCutOutLength+tolerance+FootBaseWidth*2+chamfer*2)).toCSG()
		LinkMountSideChamfer = LinkMountSideChamfer.difference(LinkMountBlank)
		CSG LinkMount = new Cube(linkWidth,(JointSpacing+linkThickness*2),(LinkMountingCutOutLength+tolerance+FootBaseWidth*2)-largeChamfer*2).toCSG()
		CSG LinkMountChamfer = StraightChamfer(linkWidth-largeChamfer, (JointSpacing+linkThickness*2)-largeChamfer, largeChamfer)
		LinkMount = LinkMount.toZMax().union(LinkMountChamfer)
		LinkMount = LinkMount.toZMin().union(LinkMountChamfer.roty(180)).moveToCenter().difference(LinkMountSideChamfer)

		CSG BoltHole = new Cylinder(mountRad, JointSpacing+linkThickness*2, 40).toCSG().moveToCenter().rotx(90)
		LinkMount = LinkMount.difference(BoltHole)

		CSG LinkCutOut = new Cube(linkWidth-(LinkMountingCutOutWidth-tolerance)*2,(linkThickness+tolerance),(LinkMountingCutOutLength+tolerance)-filletRad).toCSG().toZMin()
		CSG LinkCutOutFillet = new Cylinder(filletRad, linkThickness+tolerance, 40).toCSG().moveToCenter().rotx(90)
		LinkCutOut = LinkCutOut.union(LinkCutOutFillet.movex(((linkWidth-(LinkMountingCutOutWidth-tolerance)*2)/2)-filletRad))
		LinkCutOut = LinkCutOut.union(LinkCutOutFillet.movex(-((linkWidth-(LinkMountingCutOutWidth-tolerance)*2)/2)+filletRad)).hull()

		CSG LinkUpperRadius = InnerRadiusFillet(filletRad,(linkThickness+tolerance))
		LinkCutOut = LinkCutOut.toZMax().union(LinkUpperRadius.movex((linkWidth-(LinkMountingCutOutWidth-tolerance)*2)/2))
		LinkCutOut = LinkCutOut.union(LinkUpperRadius.rotz(180).movex(-(linkWidth-(LinkMountingCutOutWidth-tolerance)*2)/2))

		CSG CutOutTop = new ChamferedCube(linkWidth,(linkThickness+tolerance),FootBaseWidth+chamfer*2,chamfer).toCSG().toZMin()
		CSG CutOutSlicer = new Cube(linkWidth,(linkThickness+tolerance),chamfer).toCSG().toZMin()
		CSG CutOutAddition = new Cube(linkWidth,chamfer,FootBaseWidth).toCSG().toZMin()
		CutOutTop = CutOutTop.difference(CutOutSlicer)
		CutOutTop = CutOutTop.toZMax().difference(CutOutSlicer.toZMax())
		LinkCutOut = LinkCutOut.union(CutOutTop.toZMin())
		LinkCutOut = LinkCutOut.union(CutOutAddition.toYMax().movey((linkThickness+tolerance)/2))

		LinkMount = LinkMount.toZMax().toYMax().difference(LinkCutOut.toZMax().toYMax())
		LinkMount = LinkMount.toYMin().difference(LinkCutOut.toZMax().rotz(180))
		LinkMount = LinkMount.moveToCenter()

		println(linkThickness+tolerance)

		// Assemble the whole link
		CSG link = paw
		//link.setIsWireFrame(true)
		link.setColor(Color.BLUE)
		return link
	}
}
def gen= new cadGenMarcos(resinPrintServoMount,numbers,hornDiam)

//return [gen.getGearLink(),gen.getGearLinkKeepaway()]

//MobileBase mb = (MobileBase)DeviceManager.getSpecificDevice("Marcos");
//gen.setMobileBase(mb)
//DHParameterKinematics limb = gen.getByName(mb,"DummyRightFront")
//DHParameterKinematics limb2 = gen.getByName(mb,"DummyLeftFront")
//DHParameterKinematics limb3 = gen.getByName(mb,"DummyRightRear")
//DHParameterKinematics limb4 = gen.getByName(mb,"DummyLeftRear")
//return [
//	//gen.generateCad(limb,0)
//	//,gen.generateCad(limb,1),
//	gen.generateCad(limb,0),
//	gen.generateCad(limb2,0),
//	gen.generateCad(limb3,0),
//	gen.generateCad(limb4,0),
//	gen.generateBody(mb)
//]
return gen

