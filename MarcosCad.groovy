import com.google.gson.reflect.TypeToken
import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.creature.IgenerateBed
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.ChamferedCube
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.PrepForManufacturing
import eu.mihosoft.vrl.v3d.RoundedCube
import eu.mihosoft.vrl.v3d.Sphere
import eu.mihosoft.vrl.v3d.Transform
import eu.mihosoft.vrl.v3d.parametrics.LengthParameter
import javafx.scene.paint.Color
import javafx.scene.transform.Affine
import eu.mihosoft.vrl.v3d.ChamferedCylinder
import java.lang.reflect.Type

import javax.xml.transform.TransformerFactory

import org.apache.commons.math3.analysis.function.Atan
import org.apache.commons.math3.analysis.function.Sqrt
import org.apache.commons.math3.genetics.GeneticAlgorithm

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.neuronrobotics.bowlerstudio.creature.MobileBaseCadManager
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.sdk.common.DeviceManager

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

// Begin creating the resin print horn piece withn a chamfered cylendar
CSG core=  ChamferedCylinder(numbers.ServoHornDiameter/2.0,numbers.ServoHornHeight,numbers.Chamfer2)
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
// use the gear maker to generate the spline
def gears = ScriptingEngine.gitScriptRun(
		"https://github.com/madhephaestus/GearGenerator.git", // git location of the library
		"bevelGear.groovy" , // file to load
		// Parameters passed to the funcetion
		[
			numbers.ServoHornNumberofTeeth,
			// Number of teeth gear a
			numbers.ServoHornNumberofTeeth,
			// Number of teeth gear b
			numbers.ServoHornSplineHeight,
			// thickness of gear A
			numbers.ServoHornToothBaseWidth,
			// gear pitch in arc length mm
			0,
			// shaft angle, can be from 0 to 100 degrees
			0// helical angle, only used for 0 degree bevels
		]
		)
// get just the pinion of the set
CSG spline = gears.get(0)
// cut the spline from the core
CSG resinPrintServoMount=cutcore.difference(spline)
resinPrintServoMount.setColor(Color.DARKGREY)

class cadGenMarcos implements ICadGenerator,IgenerateBed{
	String url = "https://github.com/OperationSmallKat/Marcos.git"
	CSG resinPrintServoMount
	HashMap<String,Double> numbers
	LengthParameter tailLength		= new LengthParameter("Cable Cut Out Length",30,[500, 0.01])
	double endOfPassiveLinkToBolt = 4.5

	Gson gson = new GsonBuilder().disableHtmlEscaping().setPrettyPrinting().create();

	public cadGenMarcos(CSG res,HashMap<String,Double> n) {
		resinPrintServoMount=res
		numbers=n
	}
	ArrayList<CSG> cache = new ArrayList<CSG>()
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
	/**
	 * This function should generate the bed or beds or parts to be used in manufacturing If parts are
	 * to be ganged up to make print beds then this should happen here
	 *
	 * @param base the base to generate
	 * @return simulatable CAD objects
	 */
	ArrayList<CSG> arrangeBed(MobileBase base){
		println "Generating Marcos Print Bed"
		ArrayList<CSG> resin = []
		ArrayList<CSG> one = []
		ArrayList<CSG> two = []
		ArrayList<CSG> three = []
		for(CSG bit :cache) {
			def bitGetStorageGetValue = bit.getStorage().getValue("bedType")
			if(bitGetStorageGetValue.present) {
				bit=bit.prepForManufacturing()
				String name=bit.getName()
				File source=new File(ScriptingEngine.getRepositoryCloneDirectory(url).getAbsolutePath()+"/print_bed_location_"+name+".json")
				if(source.exists()) {
					//println "Loading location from "+source.getAbsolutePath()
					Type TT_mapStringString = new TypeToken<ArrayList<TransformNR>>() {
							}.getType();

					ArrayList<TransformNR> l = gson.fromJson(source.text, TT_mapStringString);
					if(l!=null&& l.size()>0) {
						TransformNR location=l.get(0)
						if(location!=null) {
							Transform csfMove = TransformFactory.nrToCSG(location)
							bit=bit.transformed(csfMove)
						}
					}
				}
				def bitGetStorageGetValueGetToString = bitGetStorageGetValue.get().toString()
				if(bitGetStorageGetValueGetToString.contentEquals("resin")) {
					resin.add(bit)
				}
				else if(bitGetStorageGetValueGetToString.contentEquals("ff-One")) {
					one.add(bit)
				}
				else if(bitGetStorageGetValueGetToString.contentEquals("ff-Two")) {
					two.add(bit)
				}
				else if(bitGetStorageGetValueGetToString.contentEquals("ff-Three")) {
					three.add(bit)
				}else {
					println "unknown bed type! "+bitGetStorageGetValueGetToString
				}
				println "Adding part to Print bed "+bitGetStorageGetValueGetToString+" "+name
			}
			else
				println "Rejecting "+bit.getName()
		}

		CSG bedThree=toBed(three ,"FF-Bed-Three")
		CSG bedTwo=toBed(two ,"FF-Bed-Two")
		CSG bedOne=toBed(one ,"FF-Bed-One")
		CSG resinBed=null
		for(int i=0;i<4;i++) {
			for (int j=0;j<4;j++) {
				double x = i*(numbers.ServoHornDiameter)
				double y = j*(numbers.ServoHornDiameter+1.0)
				try {
					int index = i*4+(j)
					if(index<resin.size()) {
						println "Adding resin horn to resin bed "+index
						CSG part =resin[index]
						CSG moved = part.movex(x).movey(y)
						if(resinBed==null)
							resinBed=moved
						else
							resinBed=resinBed.dumbUnion(moved)
					}
				}catch(Exception ex) {
					ex.printStackTrace()
				}
			}
		}

		resinBed.setName("Print-Bed-Resin-Printer")
		resinBed.setColor(Color.GREY)

		return [
			resinBed,
			bedOne,
			bedTwo,
			bedThree
		]

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
	
	CSG StraightChamfer(double x, double y, double chamferHeight) {
			CSG c1 = new Cube(x, y, chamferHeight).toCSG().movez(chamferHeight)
			CSG c2 = new Cube(x+chamferHeight,y+chamferHeight, chamferHeight).toCSG()
			CSG c3 = c1.union(c2).hull()
			return c3.difference(c2).toZMin()
		
	}
	CSG ChamferedRoundCornerLug(double x, double y,double r, double h, double chamferHeight) {
		CSG corners = ChamferedCylinder(r,h,chamferHeight)
		CSG xSec= corners.union(corners.movex(x-r*2))
		return xSec.union(xSec.movey(y-(r*2))).hull().toXMin().toYMin().movex(-x/2).movey(-y/2)
	}

	public CSG calibrationLink(double rotationCenterToBoltCenter) {
		double defaultValue = numbers.LinkLength - endOfPassiveLinkToBolt
		CSG stl= Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"DriveLink.stl"))
		double chamfer = numbers.Chamfer2
		double smallChamfer = numbers.Chamfer1
		double linkWidth = numbers.LinkWidth
		double linkRadius = linkWidth/2
		double linkThickness = numbers.LinkHeight
		double filletRad=numbers.Fillet3
		double LinkMountingCutOutWidth=numbers.LinkMountingCutOutWidth
		double blockx=rotationCenterToBoltCenter-numbers.LinkMountingCutOutLength-numbers.Tolerance+endOfPassiveLinkToBolt+filletRad
		double ServoHornRad=(numbers.ServoHornDiameter+numbers.ServoHornHoleTolerance)/2.0
		double ServoHornHeight =numbers.ServoHornHeight+numbers.LooseTolerance
		double mountHeadRad =( numbers.MountingScrewHeadDiamter+numbers.LooseTolerance)/2.0
		double mountRad=(numbers.MountingScrewDiamter+numbers.LooseTolerance)/2.0
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
		
		
		println(SetScrewChamferLength)
		println(SetScrewCutOutLength)
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
		//linkBlock=linkBlock.intersect(linkBlock.getBoundingBox().movex(-chamfer))
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

		CSG ServoHornCutoutChamfer = ChamferedCylinder(ServoHornRad+smallChamfer,ServoHornHeight+smallChamfer,smallChamfer)
				.toZMax()
				.movez(smallChamfer)
		// Idle pin cutout
		CSG ServoHornCutout = ChamferedCylinder(ServoHornRad,ServoHornHeight,smallChamfer)
				//.movez(-smallChamfer)
				.union(ServoHornCutoutChamfer)
		CSG MountHeadHoleCutoutChamfer = ChamferedCylinder(mountHeadRad+smallChamfer,linkThickness+smallChamfer,smallChamfer)
				.toZMin()
				.movez(linkThickness-smallChamfer)
				
		CSG MountHoleCutoutChamfer = ChamferedCylinder(mountRad+smallChamfer,linkThickness+smallChamfer,smallChamfer)
				.toZMax()
				.movez(smallChamfer)

		CSG boltHole = new Cylinder(mountRad, linkThickness, 20).toCSG()
		CSG boltHead = new Cylinder(mountHeadRad, linkThickness, 20).toCSG()
				.movez(linkThickness-numbers.MountingScrewHeadHeight)
		CSG mountAssebmbly = MountHoleCutoutChamfer
				.union(MountHeadHoleCutoutChamfer)
				.union(boltHole)
				.union(boltHead)
				.movex(rotationCenterToBoltCenter)

		CSG decritiveDivit = ChamferedCylinder(decritiveRad+chamfer,chamfer*2+1,chamfer)
				.movez(linkThickness-chamfer)
		CSG decoration = decorationGen(rotationCenterToBoltCenter)
		
		CSG SquareNutCutOut = new Cube(SquareNutHeight,SquareNutWidth, SquareNutCutOutHeight).toCSG()
		.toZMin()
		.movex((LinkSqaureNutSpacing+linkRadius)-(SquareNutHeight/2))
		
		CSG SquareNutChamfer = StraightChamfer(SquareNutHeight,SquareNutWidth,smallChamfer)
		.movex((LinkSqaureNutSpacing+linkRadius)-(SquareNutHeight/2))
		
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
		CSG link = lowerEnd
				.union(linkBlock)
				.hull()
				.union(mountLug)
				.difference(leftCorner)
				.difference(rightCorner)
				.difference(LowerInnerCornerChamferCutRight)
				.difference(LowerInnerCornerChamferCutLeft)
				.difference(ServoHornCutout)
				.difference(mountAssebmbly)
				.difference(decritiveDivit)
				.difference(decoration)
				.difference(SquareNutCutOut.rotz(SetScrewAngle))
				.difference(SquareNutCutOut.rotz(-SetScrewAngle))
				.difference(SetScrewCutOut.rotz(SetScrewAngle))
				.difference(SetScrewCutOut.rotz(-SetScrewAngle))
				.difference(SquareNutChamfer.rotz(SetScrewAngle))
				.difference(SetScrewChamferleft)
				.difference(SetScrewChamferright)

		//link.setIsWireFrame(true)
		link.setColor(Color.RED)
		return link//.union(stl)

		//return stl
	}
	/*
	 * public CSG test() { double defaultValue = numbers.LinkLength -
	 * endOfPassiveLinkToBolt CSG stl= Vitamins.get(ScriptingEngine.fileFromGit(
	 * "https://github.com/OperationSmallKat/Marcos.git", "DriveLink.stl")) double
	 * chamfer = numbers.Chamfer2 double smallChamfer = numbers.Chamfer1 double
	 * linkWidth = numbers.LinkWidth double linkRadius = linkWidth/2 double
	 * linkThickness = numbers.LinkHeight double filletRad=numbers.Fillet3 double
	 * LinkMountingCutOutWidth=numbers.LinkMountingCutOutWidth //double
	 * blockx=rotationCenterToBoltCenter-numbers.LinkMountingCutOutLength-numbers.
	 * Tolerance+endOfPassiveLinkToBolt+filletRad double
	 * ServoHornRad=(numbers.ServoHornDiameter+numbers.ServoHornHoleTolerance)/2.0
	 * double ServoHornHeight =numbers.ServoHornHeight+numbers.LooseTolerance double
	 * mountHeadRad =( numbers.MountingScrewHeadDiamter+numbers.LooseTolerance)/2.0
	 * double mountRad=(numbers.MountingScrewDiamter+numbers.LooseTolerance)/2.0
	 * double decritiveRad = numbers.ServoHornDiameter/4.0 double SetscrewLength =
	 * numbers.SetScrewLength double SetscrewSize = numbers.SetScrewSize double
	 * SquareNutWidth = numbers.SquareNutWidth + numbers.LooseTolerance double
	 * SquareNutHeight = numbers.SquareNutHeight + numbers.LooseTolerance double
	 * SquareNutCutOutHeight = linkThickness/2+SquareNutWidth/2 double
	 * LinkSqaureNutSpacing = numbers.LinkSqaureNutSpacing double hypot1 =
	 * Math.hypot(ServoHornRad + SetscrewLength + numbers.LooseTolerance,
	 * SetscrewSize/2) double angle1 = Math.asin(linkRadius/hypot1) double angle2 =
	 * Math.asin((ServoHornRad + SetscrewLength + numbers.LooseTolerance)/hypot1)
	 * double angle3 = (Math.PI/2)-angle1 double angle4 = (Math.PI/2)-angle2 double
	 * SetScrewAngle = Math.toDegrees((Math.PI/2)-(angle3+angle4)) double
	 * SetScrewChamferLength = linkRadius/Math.sin((Math.PI/2)-(angle3+angle4))
	 * double SetScrewCutOutLength =
	 * numbers.LinkLength/Math.cos((Math.PI/2)-(angle3+angle4))
	 * 
	 * CSG cutout1 = new Cylinder((SetscrewSize)/2, SetScrewCutOutLength).toCSG()
	 * .toZMax() .roty(90) .movez(linkThickness/2) .rotz(SetScrewAngle)
	 * 
	 * CSG cutout2 = new Cylinder((SetscrewSize)/2+chamfer,
	 * SetScrewCutOutLength/2).toCSG() .toZMax() .roty(90) .movez(linkThickness/2)
	 * .movex(linkRadius) .rotz(SetScrewAngle)
	 * 
	 * CSG Flatwall = new Cube(numbers.LinkLength,linkWidth,linkThickness).toCSG()
	 * .toZMax() .movez(linkThickness) .movex(numbers.LinkLength/2) CSG Flatwall2 =
	 * new Cube(numbers.LinkLength,linkWidth,linkThickness).toCSG() .toZMax()
	 * .movez(linkThickness) .movex(numbers.LinkLength/2) .movey(chamfer)
	 * 
	 * CSG Flatwall3 = new Cube(numbers.LinkLength,linkWidth,linkThickness).toCSG()
	 * .toZMax() .movez(linkThickness) .movex(numbers.LinkLength/2)
	 * .movey(-linkWidth) .movey(-chamfer)
	 * 
	 * CSG p1 = ((Flatwall.intersect(cutout1)).difference(Flatwall2)) CSG p2 =
	 * (cutout2.difference(Flatwall).difference(Flatwall3)) CSG link =
	 * p2.union(p1).hull()
	 * 
	 * link.setColor(Color.RED) return link//.union(stl) }
	 */
	
	


	public CSG passiveLink(double rotationCenterToBoltCenter) {
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

		CSG IdlePinCutoutChamfer = ChamferedCylinder(IdlePinRad+smallChamfer,idlePinHeight+smallChamfer,smallChamfer)
				.toZMax()
				.movez(smallChamfer)
		// Idle pin cutout
		CSG IdlePinCutout = ChamferedCylinder(IdlePinRad,idlePinHeight,smallChamfer)
				//.movez(-smallChamfer)
				.union(IdlePinCutoutChamfer)
		CSG MountHeadHoleCutoutChamfer = ChamferedCylinder(mountHeadRad+smallChamfer,linkThickness+smallChamfer,smallChamfer)
				.toZMin()
				.movez(linkThickness-smallChamfer)
		CSG MountHoleCutoutChamfer = ChamferedCylinder(mountRad+smallChamfer,linkThickness+smallChamfer,smallChamfer)
				.toZMax()
				.movez(smallChamfer)

		CSG boltHole = new Cylinder(mountRad, linkThickness, 20).toCSG()
		CSG boltHead = new Cylinder(mountHeadRad, linkThickness, 20).toCSG()
				.movez(linkThickness-numbers.MountingScrewHeadHeight)
		CSG mountAssebmbly = MountHoleCutoutChamfer
				.union(MountHeadHoleCutoutChamfer)
				.union(boltHole)
				.union(boltHead)
				.movex(rotationCenterToBoltCenter)
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
				.difference(IdlePinCutout)
				.difference(mountAssebmbly)
				.difference(decritiveDivit)
				.difference(zipTieCut)
				.difference(rightZipTie)
				.difference(decoration)

		//link.setIsWireFrame(true)
		link.setColor(Color.RED)
		return link//.union(stl)
	}
	CSG decorationGen(double rotationCenterToBoltCenter) {
		double backOffset = 4

		double chamfer = numbers.Chamfer2
		double LugDepth = numbers.LinkLength-rotationCenterToBoltCenter
		double x=rotationCenterToBoltCenter-LugDepth+chamfer-numbers.LinkDetailSize/2-backOffset
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

	@Override
	public ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {
		if(d.getScriptingName().startsWith("Head")||d.getScriptingName().startsWith("Tail")) {
			return generateCadHeadTail(d, linkIndex)
		}
		// chaeck to see if this is the left side
		boolean left=false;
		boolean front=false;
		boolean isDummyGearWrist = false;
		double parametric = numbers.LinkLength-endOfPassiveLinkToBolt
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
		double distanceToMotorTop = motor.getMaxZ()-numbers.LooseTolerance;
		println "Center to horn distance "+distanceToMotorTop
		// a list of CSG objects to be rendered
		ArrayList<CSG> back =[]
		// get the UI manipulator for the link
		Affine dGetLinkObjectManipulator = d.getLinkObjectManipulator(linkIndex)
		// UI manipulator for the root of the limb
		Affine root = d.getRootListener()

		double link1Rotz=-90
		double MototRetractDist =15
		if(linkIndex==0) {
			motor=motor.rotz(left?180:0)
			motor=motor.roty(front?180:0)
			// the first link motor is located in the body
			motor.setManipulator(root)
			// pull the limb servos out the top
			motor.addAssemblyStep(4, new Transform().movex(isDummyGearWrist?-30:MototRetractDist))
		}else {
			motor=motor.roty(left?180:0)
			motor=motor.rotz(linkIndex==2?90:90+link1Rotz)
			// the rest of the motors are located in the preior link's kinematic frame
			motor.setManipulator(d.getLinkObjectManipulator(linkIndex-1))
			// pull the link motors out the thin side

			motor.addAssemblyStep(7, new Transform().movex(linkIndex==1?MototRetractDist:0).movey(linkIndex==2?-MototRetractDist:0))
			//motor.addAssemblyStep(8, new Transform().movex(-30))
		}
		// do not export the motors to STL for manufacturing
		motor.setManufacturing({return null})
		motor.setColor(Color.BLUE)
		//Start the horn link
		// move the horn from tip of the link space, to the Motor of the last link space
		// note the hore is moved to the centerline distance value before the transform to link space
		if(!isDummyGearWrist) {
			CSG movedDrive = calibrationLink(parametric)
			.movez(distanceToMotorTop)//.rotz(180)
			double xrot=180
			xrot+=linkIndex==0&&(!front)?180:0
			xrot+=linkIndex!=0&&(!left)?180:0
			movedDrive=movedDrive.rotx(xrot)
			double zrotVal = -d.getDH_Theta(linkIndex)
			if(linkIndex==1) {
				zrotVal+=45
			}
			if(linkIndex==2) {
				zrotVal+=(-90+numbers.FootAngle)+1
			}
			movedDrive=movedDrive.rotz(zrotVal)
			CSG myDriveLink = moveDHValues(movedDrive,d,linkIndex)
			if(!isDummyGearWrist) {
				if(linkIndex==0)
					myDriveLink.addAssemblyStep(9, new Transform().movey(front?20:-20))
				else
					myDriveLink.addAssemblyStep(9, new Transform().movez(left?-20:20))
			}else {
				myDriveLink.addAssemblyStep(4, new Transform().movex(isDummyGearWrist?-30:30))

			}
			//reorent the horn for resin printing
			myDriveLink.setManufacturing({incoming ->
				return reverseDHValues(incoming.rotz(-zrotVal).rotx(-xrot), d, linkIndex).toZMin()
			})
			myDriveLink.getStorage().set("bedType", "ff-Two")
			myDriveLink.setName("DriveLink "+linkIndex+" "+d.getScriptingName())
			// attach this links manipulator
			myDriveLink.setManipulator(dGetLinkObjectManipulator)
			back.add(myDriveLink)
		}
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
			return reverseDHValues(incoming, d, linkIndex).toZMin()
					.roty(45)
					.toZMin()
					.movez(5)
		})
		myServoHorn.getStorage().set("bedType", "resin")
		myServoHorn.setName("Resin Horn "+linkIndex+" "+d.getScriptingName())
		// attach this links manipulator
		myServoHorn.setManipulator(dGetLinkObjectManipulator)
		back.add(myServoHorn)
		//end horn link

		if(isDummyGearWrist) {
			motor.addAssemblyStep(3, new Transform().movez(front?60:-60))
			myServoHorn.addAssemblyStep(3, new Transform().movey(front?-50:50))

			CSG tmp =Vitamins.get(ScriptingEngine.fileFromGit(
					"https://github.com/OperationSmallKat/Marcos.git",
					"DriveGear.stl"))
					.roty(front?180:0)
			if(front)
				tmp=tmp.toZMax()
			else
				tmp=tmp.toZMin()
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
				kneeCover.setManipulator(d.getLinkObjectManipulator(linkIndex-1))
				kneeCover.setManufacturing({incoming->
					return incoming.rotx(-90).toZMin().roty(90).toZMin()
				})
				kneeCover.getStorage().set("bedType", "ff-Two")
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
				knee.setManipulator(d.getLinkObjectManipulator(linkIndex-1))
				knee.setManufacturing({incoming->
					return incoming.rotx(-90).roty(-90).toZMin()
				})
				knee.getStorage().set("bedType", "ff-Two")
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
				knee.setName("Knee"+d.getScriptingName())
				back.add(knee)

				CSG foot = getFoot()
				foot.setManipulator(dGetLinkObjectManipulator)
				foot.setManufacturing({incoming->
					return incoming.rotx(90).roty(90-numbers.FootAngle).toZMin().rotz(front?180:0)
				})
				foot.getStorage().set("bedType", "ff-Two")
				foot.setName("Foot"+d.getScriptingName())
				back.add(foot)

			}
			double kinematicsLen = d.getDH_R(linkIndex)
			double staticOffset = 55.500-numbers.LinkLength-endOfPassiveLinkToBolt
			double calculated = kinematicsLen-staticOffset
			
			double xrot=0
			CSG link = passiveLink(parametric)
					.movez(distanceToMotorTop)
			xrot+=linkIndex==0&&(!front)?180:0
			xrot+=linkIndex!=0&&(!left)?180:0
			link=link.rotx(xrot)
			double zrotVal = -d.getDH_Theta(linkIndex)
			if(linkIndex==1) {
				zrotVal+=45
			}
			if(linkIndex==2) {
				zrotVal+=(-90+numbers.FootAngle)
			}
			link=link.rotz(zrotVal)
			CSG wrist= moveDHValues(link, d, linkIndex)
			if(linkIndex!=0)
				wrist.addAssemblyStep(10, new Transform().movez(left?5:-5))
			else
				wrist.addAssemblyStep(10, new Transform().movey(front?-5:5))

			//.rotx(90)
			wrist.setName("PassiveLink"+d.getScriptingName()+linkIndex)
			wrist.setManufacturing({ incoming ->
				return reverseDHValues( incoming.rotz(-zrotVal).rotx(-xrot), d, linkIndex).toZMin().toXMin().toYMin()
			})
			wrist.getStorage().set("bedType", "ff-Two")
			wrist.setManipulator(d.getLinkObjectManipulator(linkIndex))
			back.add(wrist)
		}
		motor.setName(conf.getElectroMechanicalSize())
		back.add(motor)
		cache.addAll(back)
		return back;
	}
	CSG getNeckLink() {
		double neckLenFudge = 4.5
		double parametric = numbers.LinkLength-endOfPassiveLinkToBolt

		return passiveLink(parametric+neckLenFudge)
				.rotx(180)
				.movez(-15.1)
	}
	public ArrayList<CSG> generateCadHeadTail(DHParameterKinematics d, int linkIndex) {
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
			back.add(wrist)
		}
		if(linkIndex==1) {
			String name= d.getScriptingName();
			CSG link = getNeckLink()

			link.addAssemblyStep(4, new Transform().movez(30))
			link.addAssemblyStep(2, new Transform().movez(-30))

			link.setName("PassiveLink"+d.getScriptingName())
			link.setManufacturing({ incoming ->
				return incoming.roty(180).toZMin().toXMin().toYMin()
			})
			link.getStorage().set("bedType", "ff-Three")
			back.add(link)
			CSG gearLink= Vitamins.get(ScriptingEngine.fileFromGit(
					"https://github.com/OperationSmallKat/Marcos.git",
					"GearLink.stl"))

					.movez(16.25)
			gearLink.addAssemblyStep(4, new Transform().movez(30))
			gearLink.addAssemblyStep(2, new Transform().movez(30))

			gearLink.setName("GearLink"+d.getScriptingName())
			gearLink.setManufacturing({ incoming ->
				return incoming.roty(180).toZMin().toXMin().toYMin()
			})
			gearLink.getStorage().set("bedType", "ff-One")
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
					return incoming.toZMin().toXMin().toYMin()
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
			back.add(headtail)


		}

		for(CSG c:back)
			c.setManipulator(d.getLinkObjectManipulator(linkIndex))
		cache.addAll(back)
		return back;
	}

	CSG getFoot() {
		CSG foot  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"Foot.stl"))
				.rotx(180)
	}
	DHParameterKinematics getByName(MobileBase b,String name) {
		for(DHParameterKinematics k:b.getAllDHChains()) {
			if(k.getScriptingName().contentEquals(name))
				return k
		}
		return null;
	}
	@Override
	public ArrayList<CSG> generateBody(MobileBase arg0) {
		cache.clear()
		DHParameterKinematics dh = arg0.getLegs().get(0);

		double zCenterLine = dh.getRobotToFiducialTransform().getZ()+numbers.ServoThickness/2.0

		CSG body  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"Body.stl")).movez(zCenterLine);
		CSG bodyCOver  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"BodyCover.stl")).movez(zCenterLine);
		CSG topCOver  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"BodyServoCoverTop.stl")).movez(zCenterLine);
		CSG BottomCover  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"BodyCoverBottom.stl")).movez(zCenterLine);
		ArrayList<CSG> back =[body, bodyCOver]
		for(CSG c:back) {
			c.getStorage().set("bedType", "ff-One")
		}
		for(DHParameterKinematics k:arg0.getLegs()) {
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
			bottom.getStorage().set("bedType", "ff-Two")
			top.addAssemblyStep(6, new Transform().movez(10))
			bottom.addAssemblyStep(6, new Transform().movez(-10))
			double distacne = front?80:-80
			top.addAssemblyStep(5, new Transform().movey(distacne))
			bottom.addAssemblyStep(5, new Transform().movey(distacne))

			back.addAll([top, bottom])
			println "ServoCover's for "+(left?"Left":"Right")+(front?"Front":"Back")
		}
		// Set the location of the limbs based on the CSV in the body loader
		for(DHParameterKinematics d:arg0.getAllDHChains()) {
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
				if(arg0.getLegs().contains(d)) {
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
		bodyCOver.addAssemblyStep(5, new Transform().movez(80))
		body.setName("Body")
		body.setManufacturing({ incoming ->
			return incoming.rotx(180).toZMin().toXMin().toYMin()
		})
		bodyCOver.setManufacturing({ incoming ->
			return incoming.toZMin().toXMin().toYMin().movey(body.getTotalY()+1)
		})

		Transform tipLeftFront = TransformFactory.nrToCSG(getByName(arg0,"LeftFront").calcHome())
		Transform tipRightFront = TransformFactory.nrToCSG(getByName(arg0,"RightFront").calcHome())
		Transform tipLeftRear = TransformFactory.nrToCSG(getByName(arg0,"LeftRear").calcHome())
		Transform tipRightRear = TransformFactory.nrToCSG(getByName(arg0,"RightRear").calcHome())

		Transform neck =TransformFactory.nrToCSG(getByName(arg0,"Head").calcHome())
		Transform butt =TransformFactory.nrToCSG(getByName(arg0,"Tail").calcHome())

		CSG neckBit = getNeckLink().transformed(neck)
		CSG buttBit = getNeckLink().transformed(butt)

		CSG calBlock = new ChamferedCube(25,25,20,numbers.Chamfer2).toCSG()
				.toZMin()
				.movez(5)
		CSG calLeft =calBlock.toYMin().movey(2)
		CSG calRight = calBlock.toYMax().movey(-2)
		CSG footLeftFront=getFoot().transformed(tipLeftFront)
		CSG footRightFront=getFoot().transformed(tipRightFront)
		CSG footLeftRear=getFoot().transformed(tipLeftRear)
		CSG footRightRear=getFoot().transformed(tipRightRear)

		CSG fCenter=calBlock.move(tipLeftFront.x, 0, tipLeftFront.z)
		CSG rCenter=calBlock.move(tipRightRear.x, 0, tipRightRear.z)
		CSG Center = fCenter
				.union(rCenter)
				.hull()
		double calSinkInDistance =4
		CSG fCal = calBlock.toZMax().move(neck.x, neck.y, neckBit.getMinZ()+numbers.Chamfer2+calSinkInDistance)
				.union(fCenter)
				.hull()
				.difference(neckBit)
		CSG rCal = calBlock.toZMax().move(butt.x, butt.y, buttBit.getMinZ()+numbers.Chamfer2+calSinkInDistance)
				.union(rCenter)
				.hull()
				.difference(buttBit)

		CSG FrontSpar = calBlock.move(tipLeftFront.x, tipLeftFront.y, tipLeftFront.z)
				.union(calBlock.move(tipRightFront.x, tipRightFront.y, tipRightFront.z))
				.hull()
				.difference(footLeftFront)
				.difference(footRightFront)
		CSG RearSpar = calBlock.move(tipLeftRear.x, tipLeftRear.y, tipLeftRear.z)
				.union(calBlock.move(tipRightRear.x, tipRightRear.y, tipRightRear.z))
				.hull()
				.difference(footLeftRear)
				.difference(footRightRear)
		CSG spars = Center.union([
			FrontSpar,
			RearSpar,
			fCal,
			rCal
		])
		//		CSG LeftFrontbox=calBlock.move(tipLeftFront.x, tipLeftFront.y, tipLeftFront.z).difference(footLeftFront)
		//		CSG RightFrontbox=calBlock.move(tipRightFront.x, tipRightFront.y, tipRightFront.z).difference(footRightFront)
		//		CSG LeftRearbox=calBlock.move(tipLeftRear.x, tipLeftRear.y, tipLeftRear.z).difference(footLeftRear)
		//		CSG RightRearbox=calBlock.move(tipRightRear.x, tipRightRear.y, tipRightRear.z).difference(footRightRear)
		spars.setName("CalibrationJig")
		spars.getStorage().set("bedType", "ff-Three")

		spars.setManufacturing({incoming -> return incoming.toZMin()})

		back.addAll([spars])

		for(CSG c:back) {
			c.setManipulator(arg0.getRootListener())
		}
		//		for(DHParameterKinematics kin:arg0.getAllDHChains()) {
		//			CSG limbRoot =new Cube(1).toCSG()
		//			limbRoot.setManipulator(kin.getRootListener())
		//			back.add(limbRoot)
		//		}
		cache.addAll(back)
		return back;
	}


}
def gen= new cadGenMarcos(resinPrintServoMount,numbers)

return [gen.calibrationLink(32-4.5)]
//return [gen.test()]
// test
return gen

