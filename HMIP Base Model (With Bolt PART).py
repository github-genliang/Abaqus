#*********************************************************************
#HMIP Fillet Welded Socket Connection
#Compare to Stam Base Model & Figure 6.2
#Baseplate: 3"x36" OD
#Shaft: 5/16"x24" OD, 16 sided
#Anchor rods: 1.88" Dia., 30" ring
#Weld: Inside, .31" Fillet; Outside, .62x.37 fillet
#*********************************************************************
bpr = 18.0 #Base plate outer radius (inches)
bpt = 3.0 #Base plate thickness (inches)
shaftt = .3125 #Shaft thickness (inches)
shaftd = 24.0 #Shaft diameter (inches)
shaftr = shaftd/2.0 #shaft radius (inches)
shafts = 16.0 #Number of shaft sides
shaftl = 12.0*2 #Solid shaft length (inches)
shelll = 12.0*14 #Shell shaft length (inches)
bendrad = 4.1325 #Shaft bend radius
num_ar = 8 #Number of anchor rods
ard = 1.875 #Anchor rod diameter (inches)
nd = 2.625 #Nut diameter
arradial = 30.0 #Diameter of anchor rod radial placement (inches)
iweld = .3125 #Inner weld (inches)
oweldl = .62 #Outer weld length (inches)
oweldw = .37 #Outer weld width (inches)

from abaqus import *
from abaqusConstants import *
import regionToolset
import itertools

# -*- coding: mbcs -*-
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *

#----------------------------------------------------------------------
#Create the model
mdb.models.changeKey(fromName='Model-1', toName='HMIP Base Model')
baseModel = mdb.models['HMIP Base Model']

#----------------------------------------------------------------------
#Create the parts
#Base Plate - Half circle and create datum axis
bps = baseModel.ConstrainedSketch(name='BasePlateCS',sheetSize=50.0)
bps.Arc3Points(point1=(0, bpr),point3=(-bpr,0),point2=(0, -bpr))
bps.Line(point1=(0,-bpr),point2=(0,bpr))
bp = baseModel.Part(name='Base Plate', dimensionality=THREE_D,type=DEFORMABLE_BODY)
bp.BaseSolidExtrude(sketch=bps, depth=bpt)

bp.DatumAxisByPrincipalAxis(principalAxis=YAXIS)
datum_keys = bp.datums.keys()
ydatum = bp.datums[datum_keys[0]]
bpface = bp.faces.findAt(((0.0,0.0,0),))

#Polygonal shape of shaft and welds
polygon = baseModel.ConstrainedSketch(name='Polygon', sheetSize=50,)   
t = 0
polypointsx = []
polypointsy = []
polylines = []
while t<=(shafts/2+2):
    polypointsx.append((shaftr*cos(2*pi/shafts*(t-1)+pi/2)))
    polypointsy.append((shaftr*sin(2*pi/shafts*(t-1)+pi/2)))
    t = t+1
t = 0
while t<(shafts/2+2):
    polygon.Line((polypointsx[t],polypointsy[t]),(polypointsx[t+1],polypointsy[t+1]))
    t = t+1
t=0
while t<=(shafts/2+1):
    polylines.append(polygon.geometry.findAt(((polypointsx[t]+polypointsx[t+1])/2,
                                           (polypointsy[t]+polypointsy[t+1])/2)))
    t = t+1
t = 0
while t<=(shafts/2): #Fillet the sharp edges
    polygon.FilletByRadius(curve1=polylines[t], curve2=polylines[t+1], 
    nearPoint1=(((polypointsx[t]+polypointsx[t+1])/2,(polypointsy[t]+polypointsy[t+1])/2)), 
    nearPoint2=(((polypointsx[t+1]+polypointsx[t+2])/2,(polypointsy[t+1]+polypointsy[t+2])/2)),
    radius=bendrad)
    t = t+1
mylist = []
for key in polygon.geometry.keys(): #Next few lines, remove excess geometry
    mylist.append(polygon.geometry[key])
c1 = mylist[len(polylines)]
c2 = mylist[len(mylist)-1]
pt1 = c1.getPointAtDistance(point=c1.getVertices()[0].coords,
     distance=50,percentage=1)
pt2 = c2.getPointAtDistance(point=c2.getVertices()[0].coords,
     distance=50,percentage=1)
l1 = polygon.Line(point1=pt1,point2=pt2)
polygon.delete((polylines[0],polylines[len(polylines)-1]))
polygon.trimExtendCurve(curve1=c1,curve2=l1, point1=c1.getVertices()[1].coords,
     point2=(0,shaftr-2))
polygon.trimExtendCurve(curve1=c2,curve2=l1, point1=c2.getVertices()[0].coords,
     point2=(0,-shaftr+2))

#Boltholes in Base Plate
bpface = bp.faces.findAt(((0.0,0.0,0),))
bp.CutExtrude(sketch=polygon, sketchPlane=bpface[0],sketchPlaneSide=SIDE2,
              sketchUpEdge=ydatum,sketchOrientation=LEFT,flipExtrudeDirection=ON)
hole = baseModel.ConstrainedSketch(name='Hole', sheetSize=50,)  
for i in range(num_ar/2):
    xcen = arradial/2*cos(2*(i+.5)*pi/num_ar+pi/2)
    ycen = arradial/2*sin(2*(i+.5)*pi/num_ar+pi/2)                      
    hole.CircleByCenterPerimeter(center=(xcen,ycen),point1=(xcen,ycen+ard/2))
bpface = bp.faces.findAt(((0.0,bpr-.1,0),))
bp.CutExtrude(sketch=hole, sketchPlane=bpface[0],sketchPlaneSide=SIDE2,
              sketchUpEdge=ydatum,sketchOrientation=LEFT,flipExtrudeDirection=ON)
              
polygon.delete((l1,),)
#Inner Weld
iweldSk = baseModel.ConstrainedSketch(name='InnerWeld', sheetSize=50,)
iweldSk.Line((0,0),(-iweld,0))
iweldSk.Line((-iweld,0),(0,-iweld))
iweldSk.Line((0,-iweld),(0,0))
iw = baseModel.Part(name='Inner Weld', dimensionality=THREE_D,type=DEFORMABLE_BODY)
iw.BaseSolidSweep(path=polygon, sketch=iweldSk)

#Outer Weld
oweldSk = baseModel.ConstrainedSketch(name='OuterWeld', sheetSize=50,)
oweldSk.Line((0,0),(0,oweldl))
oweldSk.Line((0,oweldl),(oweldw,0))
oweldSk.Line((oweldw,0),(0,0))
ow = baseModel.Part(name='Outer Weld', dimensionality=THREE_D,type=DEFORMABLE_BODY)
ow.BaseSolidSweep(path=polygon, sketch=oweldSk)

#Anchor Rods
arSk = baseModel.ConstrainedSketch(name='AnchorRodCS',sheetSize=50.0)
bolts = arSk.CircleByCenterPerimeter(center=(arradial/2*cos(pi/2+pi/num_ar), 
    arradial/2*sin(pi/2+pi/num_ar)),point1=(arradial/2*cos(pi/2+pi/num_ar),
    arradial/2*sin(pi/2+pi/num_ar)+ard/2))
rod = baseModel.Part(name='Rod', dimensionality=THREE_D,type=DEFORMABLE_BODY)
rod.BaseSolidExtrude(sketch=arSk, depth=5)

nutSk = baseModel.ConstrainedSketch(name='NutsCS',sheetSize=50.0)
holes = nutSk.CircleByCenterPerimeter(center=(arradial/2*cos(pi/2+pi/num_ar), 
    arradial/2*sin(pi/2+pi/num_ar)),point1=(arradial/2*cos(pi/2+pi/num_ar),
    arradial/2*sin(pi/2+pi/num_ar)+nd/2))
nut = baseModel.Part(name='nut', dimensionality=THREE_D,type=DEFORMABLE_BODY)
nut.BaseSolidExtrude(sketch=nutSk, depth=1.5)
for i in range(num_ar/2):
    xcen = arradial/2*cos(2*(i+.5)*pi/num_ar+pi/2)
    ycen = arradial/2*sin(2*(i+.5)*pi/num_ar+pi/2)                      
    nutSk.CircleByCenterPerimeter(center=(xcen,ycen),point1=(xcen,ycen+nd/2))

#Solid Shaft
shaftSk = baseModel.ConstrainedSketch(name='ShaftCS',sheetSize=50.0)
shaftSk.retrieveSketch(sketch=polygon) 
mylist = []
for key in shaftSk.geometry.keys():
    mylist.append(shaftSk.geometry[key])
shaftSk.Line(point1=pt1,point2=(0,pt1[1]-shaftt))
shaftSk.Line(point1=pt2,point2=(0,pt2[1]+shaftt))
mytup = tuple(mylist)
shaftSk.offset(distance=shaftt,objectList=mytup,side=LEFT)
shaft = baseModel.Part(name='Shaft', dimensionality=THREE_D,type=DEFORMABLE_BODY)
shaft.BaseSolidExtrude(sketch=shaftSk, depth=shaftl)
#Shell Shaft
shellSk = baseModel.ConstrainedSketch(name='ShellCS',sheetSize=50.0)
shellSk.retrieveSketch(sketch=polygon) 
mylist = []
for key in shellSk.geometry.keys():
    mylist.append(shellSk.geometry[key])
mytup = tuple(mylist)
shellSk.scale(scaleValue=((shaftr-shaftt/2)/shaftr),objectList=mytup,scaleCenter=(0.0,0.0))
shell = baseModel.Part(name='ShellShaft', dimensionality=THREE_D,type=DEFORMABLE_BODY)
shell.BaseShellExtrude(sketch=shellSk, depth=shelll)

#End plate
endSk = baseModel.ConstrainedSketch(name='EndPlateCS',sheetSize=50.0)
endSk.retrieveSketch(sketch=shellSk) 
endSk.Line(point1 = (0,pt1[1]*(shaftr-shaftt/2)/shaftr), point2 = (0,-pt1[1]*(shaftr-shaftt/2)/shaftr))
ep = baseModel.Part(name='End Plate', dimensionality=THREE_D,type=DEFORMABLE_BODY)
ep.BaseShell(sketch=endSk)

#----------------------------------------------------------------------
#Create assembly

#Create the part instances and assemble
ass = baseModel.rootAssembly
bpInst = assemblie.Instance(name='BP Instance', part=bp)
sInst = assemblie.Instance(name='Shaft Instance', part=shaft)
iwInst = assemblie.Instance(name='IW Instance', part=iw)
owInst = assemblie.Instance(name='OW Instance', part=ow)
poInst = assemblie.Instance(name='Shell Instance', part=shell)
spInst = assemblie.Instance(name='EP Instance', part=ep)
rodInst = assemblie.Instance(name='Rod Instance', part=rod)
nutInst = assemblie.Instance(name='Nut Instance', part=nut)

#Assemble
assemblie.translate(instanceList=('Shaft Instance',),vector=(0,0,bpt/2))
assemblie.translate(instanceList=('IW Instance',),vector=(0,0,(bpt/2)))
assemblie.translate(instanceList=('OW Instance',),vector=(0,0,bpt))
assemblie.translate(instanceList=('EP Instance',),vector=(0,0,bpt/2+shaftl+shelll))
assemblie.translate(instanceList=('Shell Instance',),vector=(0,0,bpt/2+shaftl))
assemblie.translate(instanceList=('Nut Instance',),vector=(0,0,bpt))
assemblie.translate(instanceList=('Rod Instance',),vector=(0,0,-2))
baseInst = assemblie.InstanceFromBooleanMerge(domain=GEOMETRY, instances=(assemblie.instances['BP Instance'],
    assemblie.instances['Shaft Instance'], assemblie.instances['OW Instance'],assemblie.instances['IW Instance']),
    keepIntersections=ON, name='MergedSolid', originalInstances=SUPPRESS)
shInst = assemblie.InstanceFromBooleanMerge(domain=GEOMETRY, instances=(assemblie.instances['EP Instance'], 
    assemblie.instances['Shell Instance']), name='MergedShell', originalInstances=SUPPRESS)
arInst = assemblie.InstanceFromBooleanMerge(domain=GEOMETRY, instances=(assemblie.instances['Rod Instance'], 
    assemblie.instances['Nut Instance']), name='AnchorRods', originalInstances=SUPPRESS)
arInst2 = assemblie.RadialInstancePattern(instanceList=('AnchorRods-1',),number=4, 
                          totalAngle=180.0*(num_ar/2-1)/(num_ar/2))
assemblie.makeIndependent(instances=(baseInst,))
assemblie.makeIndependent(instances=(shInst,))


#----------------------------------------------------------------------
#Create material

# Create material Steel
steelm = baseModel.Material(name='Steel')
steelm.Elastic(table=((29000, 0.3), )) #ksi

#----------------------------------------------------------------------
#Create solid section and assign 
bs = baseModel.HomogeneousSolidSection(name='Steel Section', 
    material='Steel')
p = baseModel.parts['MergedSolid']
region = regionToolset.Region(cells=p.cells.getByBoundingBox(xMin=-bpr,xMax=bpr,
                              yMin=-bpr,yMax=bpr,zMin=0,zMax=shaftl*2))
p.SectionAssignment(region=region, sectionName='Steel Section', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='',thicknessAssignment=FROM_SECTION)
p = baseModel.parts['AnchorRods']
region = regionToolset.Region(cells=p.cells.getByBoundingBox(xMin=-bpr,xMax=bpr,
                              yMin=-bpr,yMax=bpr,zMin=-5,zMax=shaftl*2))
p.SectionAssignment(region=region, sectionName='Steel Section', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='',thicknessAssignment=FROM_SECTION)
                              
#Create shell section for shaft & ep
bsh = baseModel.HomogeneousShellSection(name='Shell Section', 
    material='Steel',integrationRule=SIMPSON, nodalThicknessField='', numIntPts=5, 
    poissonDefinition=DEFAULT, preIntegrate=OFF, temperature=GRADIENT, 
    thickness=shaftt, thicknessModulus=None, useDensity=OFF)

bsep = baseModel.HomogeneousShellSection(name='EP Shell Section', 
    material='Steel',integrationRule=SIMPSON, nodalThicknessField='', numIntPts=5, 
    poissonDefinition=DEFAULT, preIntegrate=OFF, temperature=GRADIENT, 
    thickness=1000, thicknessModulus=None, useDensity=OFF)
#Shell sections
p = mdb.models['HMIP Base Model'].parts['MergedShell']
shellfaces = p.faces.getByBoundingBox(xMin=(-shaftr),xMax=1.0,
           yMin=(-shaftr),yMax=shaftr,zMin=bpt/2+shaftl,zMax=bpt/2+shaftl+shelll)
region = p.Set(faces=shellfaces, name='Shellfaces')
p.SectionAssignment(region=region, sectionName='Shell Section', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='',thicknessAssignment=FROM_SECTION)
epfaces = p.faces.getByBoundingBox(xMin=(-shaftr),xMax=1.0,
           yMin=(-shaftr),yMax=shaftr,zMin=shaftl+shelll,zMax=bpt+shaftl+shelll)
region = p.Set(faces=shellfaces, name='EPfaces')
p.SectionAssignment(region=region, sectionName='EP Shell Section', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='',thicknessAssignment=FROM_SECTION)

#Identify features for Coupling
shafttop_face = baseInst.faces.getByBoundingBox(xMin=(-shaftr),xMax=1.0,
           yMin=(-shaftr),yMax=shaftr,zMin=shaftl,zMax=(shaftl+bpt))
shelledges=shInst.edges.getByBoundingBox(xMin=(-shaftr),xMax=1.0,
           yMin=(-shaftr),yMax=shaftr,zMin=shaftl,zMax=(shaftl+bpt))
region1 = assemblie.Surface(side1Edges=shelledges,name='ShaftBotEdges')
region2 = assemblie.Surface(side1Faces=shafttop_face,name='ShaftTopFace')
baseModel.ShellSolidCoupling(name='Coupling',
    positionToleranceMethod=COMPUTED, shellEdge=region1, solidFace=region2)

#----------------------------------------------------------------------
#Create steps
baseModel.StaticStep(name='Bolt', previous='Initial',description='Bolt', 
    stabilizationMagnitude=0.0002, stabilizationMethod=DAMPING_FACTOR, 
    continueDampingFactors=False, adaptiveDampingRatio=0.05, initialInc=0.01, 
    maxInc=0.1,nlgeom=OFF)
baseModel.StaticStep(name='Load', previous='Bolt',description='Load',nlgeom=OFF)
    
#----------------------------------------------------------------------
#Interactions
baseModel.ContactProperty('friction')
baseModel.interactionProperties['friction'].TangentialBehavior(
    formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, 
    pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, table=((
    0.5, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, 
    fraction=0.005, elasticSlipStiffness=None)
baseModel.interactionProperties['friction'].NormalBehavior(pressureOverclosure=HARD,
    allowSeparation=ON,constraintEnforcementMethod=DEFAULT)
baseModel.ContactProperty('frictionless')
baseModel.interactionProperties['frictionless'].TangentialBehavior(
    formulation=FRICTIONLESS)
baseModel.interactionProperties['frictionless'].NormalBehavior(allowSeparation=ON, 
    constraintEnforcementMethod=DEFAULT,pressureOverclosure=HARD)
baseModel.StdContactControl(name='ContCtrl',stabilizeChoice=AUTOMATIC)
    
#Nuts and BP
xcen = arradial/2*cos(pi/2+pi/num_ar)
ycen = arradial/2*sin(pi/2+pi/num_ar)
mface = baseInst.faces.findAt(((-.1,bpr-.1,bpt),),)
region1 = assemblie.Surface(side1Faces=mface, name='BPTop')
sface = arInst.faces.findAt(((xcen+ard*.6,ycen,bpt),),)
region2=assemblie.Surface(side1Faces=sface, name=('BoltBot0'))
baseModel.SurfaceToSurfaceContactStd(name=('Ar0'), 
    createStepName='Initial', master=region1, slave=region2, sliding=SMALL, 
    thickness=ON, interactionProperty='friction', adjustMethod=NONE, 
    initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
mface = baseInst.faces.findAt(((xcen+ard/2,ycen,bpt/2),),)
region1=assemblie.Surface(side1Faces=mface, name=('BPH0'))
sface = arInst.faces.findAt(((xcen+ard/2,ycen,bpt/2),),)
region2=assemblie.Surface(side1Faces=sface, name=('BoltSide0'))
baseModel.SurfaceToSurfaceContactStd(name=('ArH0'), 
    createStepName='Initial', master=region2, slave=region1, sliding=SMALL, 
    thickness=ON, interactionProperty='frictionless', adjustMethod=NONE, 
    initialClearance=OMIT, datumAxis=None, clearanceRegion=None)

for i in range(1,num_ar/2):
    xcen = arradial/2*cos(pi/2+2*pi/num_ar*(i+.5))
    ycen = arradial/2*sin(pi/2+2*pi/num_ar*(i+.5))
    mface = baseInst.faces.findAt(((-.1,bpr-.1,bpt),),)
    region1 = assemblie.Surface(side1Faces=mface, name='BPTop')
    sface = arInst2[i-1].faces.findAt(((xcen+ard*.6,ycen,bpt),),)
    region2=assemblie.Surface(side1Faces=sface, name=('BoltBot'+str(i)))
    baseModel.SurfaceToSurfaceContactStd(name=('Ar'+str(i)), 
        createStepName='Initial', master=region1, slave=region2, sliding=SMALL, 
        thickness=ON, interactionProperty='friction', adjustMethod=NONE, 
        initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
    mface = baseInst.faces.findAt(((xcen+ard/2,ycen,bpt/2),),)
    region1=assemblie.Surface(side1Faces=mface, name=('BPH'+str(i)))
    sface = arInst2[i-1].faces.findAt(((xcen+ard/2,ycen,bpt/2),),)
    region2=assemblie.Surface(side1Faces=sface, name=('BoltSide'+str(i)))
    baseModel.SurfaceToSurfaceContactStd(name=('ArH'+str(i)), 
        createStepName='Initial', master=region2, slave=region1, sliding=SMALL, 
        thickness=ON, interactionProperty='frictionless', adjustMethod=NONE, 
        initialClearance=OMIT, datumAxis=None, clearanceRegion=None)

###----------------------------------------------------------------------
##Partition & Mesh Bolt
p = baseModel.parts['AnchorRods']
xcen = arradial/2*cos(pi/2+pi/num_ar)
ycen = arradial/2*sin(pi/2+pi/num_ar)
#Partition for bolt load
p.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=bpt/2)
p.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=bpt)
datkeys = p.datums.keys()
pickedCells = p.cells.getByBoundingBox(xMin=xcen-nd/2,xMax=xcen+nd/2,
                                       yMin=ycen-nd/2,yMax=ycen+nd/2,
                                       zMin=bpt-5, zMax=bpt+2)
p.PartitionCellByDatumPlane(datumPlane=p.datums[datkeys[-2]], cells=pickedCells)
pickedCells = p.cells.getByBoundingBox(xMin=xcen-nd/2,xMax=xcen+nd/2,
                                       yMin=ycen-nd/2,yMax=ycen+nd/2,
                                       zMin=bpt-5, zMax=bpt+2)
p.PartitionCellByDatumPlane(datumPlane=p.datums[datkeys[-1]], cells=pickedCells)

#Circles
p.DatumAxisByPrincipalAxis(principalAxis=ZAXIS)
datkeys = p.datums.keys()
tform = p.MakeSketchTransform(sketchPlane=p.faces.findAt((xcen,ycen,bpt+1.5),),
                              origin=(0.0,0.0,bpt+1.5))
s1 = baseModel.ConstrainedSketch(name='partition',sheetSize=50,gridSpacing=1,transform=tform)
s1.retrieveSketch(sketch=hole)
s1.CircleByCenterPerimeter(center=(xcen,ycen),point1=(xcen,ycen+ard/4))
sp = p.faces.getByBoundingBox(xMin=-bpr,xMax=0,yMin=0,yMax=bpr,zMin=bpt+1.5,zMax=bpt+1.5)
p.PartitionFaceBySketch(faces=sp,sketch=s1)

pickedCells = p.cells.findAt((xcen,ycen,bpt+1),)
pickedEdges = p.edges.findAt((xcen,ycen+ard/2,bpt+1.5),)
p.PartitionCellByExtrudeEdge(line=p.datums[datkeys[-1]], cells=pickedCells,
                                 edges=(pickedEdges,),sense=REVERSE)
pickedCells = p.cells.findAt((xcen,ycen,bpt+1),)
pickedEdges = p.edges.findAt((xcen,ycen+ard/4,bpt+1.5),)
p.PartitionCellByExtrudeEdge(line=p.datums[datkeys[-1]], cells=pickedCells,
                                 edges=(pickedEdges,),sense=REVERSE)
                                 
p.setMeshControls(elemShape=WEDGE, regions=p.cells.findAt(((xcen,ycen,bpt+1),),), 
                  technique=SWEEP)
pedges = p.edges.getByBoundingBox(xMin=-bpr,xMax=0,yMin=0,yMax=bpr,zMin=bpt+1.5,zMax=bpt+1.5)
p.seedEdgeByNumber(edges=pedges, number=16, constraint=FINER)

#Crosses
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=xcen)
p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=ycen)
datkeys = p.datums.keys()
pickedCells = p.cells.getByBoundingBox(xMin=xcen-nd/2,xMax=xcen+nd/2,
                                       yMin=ycen-nd/2,yMax=ycen+nd/2,
                                       zMin=bpt, zMax=bpt+2)
p.PartitionCellByDatumPlane(datumPlane=p.datums[datkeys[-2]], cells=pickedCells)
pickedCells = p.cells.getByBoundingBox(xMin=xcen-nd/2,xMax=xcen+nd/2,
                                       yMin=ycen-nd/2,yMax=ycen+nd/2,
                                       zMin=bpt, zMax=bpt+2)
p.PartitionCellByDatumPlane(datumPlane=p.datums[datkeys[-1]], cells=pickedCells)

pedges = p.edges.findAt(((xcen,ycen+ard/5,bpt+1.5),),((xcen,ycen-ard/5,bpt+1.5),),
                        ((xcen+ard/5,ycen,bpt+1.5),),((xcen-ard/5,ycen,bpt+1.5),),)
p.seedEdgeByNumber(edges=pedges, number=1, constraint=FIXED)
pedges = p.edges.findAt(((xcen,ycen+ard/3,bpt+1.5),),((xcen,ycen-ard/3,bpt+1.5),),
                        ((xcen+ard/3,ycen,bpt+1.5),),((xcen-ard/3,ycen,bpt+1.5),),
                        ((xcen,ycen+ard*5/8,bpt+1.5),),((xcen,ycen-ard*5/8,bpt+1.5),),
                        ((xcen+ard*5/8,ycen,bpt+1.5),),((xcen-ard*5/8,ycen,bpt+1.5),),)
p.seedEdgeByNumber(edges=pedges, number=2, constraint=FIXED)

elemType1 = ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
elemType2 = ElemType(elemCode=C3D15, elemLibrary=STANDARD)
elemType3 = ElemType(elemCode=C3D10, elemLibrary=STANDARD)
pickedCells = p.cells.getByBoundingBox(xMin=xcen-nd/2,xMax=xcen+nd/2,
                                       yMin=ycen-nd/2,yMax=ycen+nd/2,
                                       zMin=bpt-5, zMax=bpt+2)
p.setElementType(regions=(pickedCells,), elemTypes=(elemType1, elemType2, 
    elemType3))
p.generateMesh()

##Partition Base Plate
#Set element type
assemblie.setMeshControls(elemShape=WEDGE, regions=
    baseInst.cells.findAt(((0,shaftr-shaftt/3,bpt/2-iweld/3),),), technique=SWEEP)
assemblie.setMeshControls(elemShape=WEDGE, regions=
    baseInst.cells.findAt(((0,shaftr+shaftt/3,bpt+oweldw/3),),), technique=SWEEP)
elemType1 = ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
elemType2 = ElemType(elemCode=C3D15, elemLibrary=STANDARD)
elemType3 = ElemType(elemCode=C3D10, elemLibrary=STANDARD)
cells1 = baseInst.cells.getByBoundingBox(xMin=-bpr,xMax=1,yMin=-bpr,yMax=bpr,
                                         zMin=-1,zMax=bpt+shaftl)
pickedRegions =(cells1, )
assemblie.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
    elemType3))
    
#Base Seed
bpedges = baseInst.edges.getByBoundingBox(xMin=(-bpr),xMax=1.0,
           yMin=(-bpr),yMax=bpr,zMin=0,zMax=(bpt))
assemblie.seedEdgeBySize(edges=bpedges, size=shaftt*2.5, deviationFactor=0.1, 
    minSizeFactor=0.1, constraint=FINER)                                     

#Partition BoltHoles
sp = baseInst.faces.findAt(((-.1,bpr-.1,0),),)
assemblie.PartitionFaceBySketch(faces=sp, sketch=nutSk)

t = 0
while t<(num_ar/2):
    pickedCells = baseInst.cells.findAt((-.1,bpr-.1,bpt/2),)
    spath = baseInst.edges.findAt((0,bpr,bpt/2),)
    xcen = (arradial/2)*cos(2*pi/num_ar*(t+.5)+pi/2)
    ycen = (arradial/2)*sin(2*pi/num_ar*(t+.5)+pi/2)
    pickedEdges = baseInst.edges.findAt((xcen+nd/2,ycen,0),)
    assemblie.PartitionCellBySweepEdge(sweepPath=spath, cells=pickedCells,
                                 edges=(pickedEdges,))
    #Cross partition                   
    circcell = baseInst.cells.getByBoundingBox(xMin=xcen-nd/2,xMax=xcen+nd/2,
                                        yMin=ycen-nd/2,yMax=ycen+nd/2,
                                        zMin=0, zMax=bpt)
    assemblie.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=xcen)
    datkeys = assemblie.datums.keys()
    assemblie.PartitionCellByDatumPlane(datumPlane=assemblie.datums[datkeys[-1]], cells=circcell)
    pedges = baseInst.edges.findAt(((xcen+ard/2,ycen,bpt),),((xcen+nd/2,ycen,bpt),),
                        ((xcen-ard/2,ycen,bpt),),((xcen-nd/2,ycen,bpt),),)
    assemblie.seedEdgeByNumber(edges=pedges, number=7, constraint=FIXED)
    t = t+1

#Partition layers
assemblie.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=bpt/2-iweld)
assemblie.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=bpt/2)
assemblie.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=bpt)
assemblie.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=bpt+oweldl)
last = len(assemblie.datums.keys())
dats = assemblie.datums.keys()[last-4:last]
bpCells = baseInst.cells.findAt((0,bpr-shaftt,bpt/2+iweld,),)
assemblie.PartitionCellByDatumPlane(datumPlane=assemblie.datums[dats[0]], cells=bpCells)
bpCells = baseInst.cells.findAt((0,bpr-shaftt,bpt/2+iweld,),)
assemblie.PartitionCellByDatumPlane(datumPlane=assemblie.datums[dats[1]], cells=bpCells)
shaftCells = baseInst.cells.findAt((0,shaftr-shaftt/2,bpt+1,),)
assemblie.PartitionCellByDatumPlane(datumPlane=assemblie.datums[dats[2]], cells=shaftCells)
shaftCells = baseInst.cells.findAt((0,shaftr-shaftt/2,bpt+1,),)
assemblie.PartitionCellByDatumPlane(datumPlane=assemblie.datums[dats[3]], cells=shaftCells)

#Partition End Plate
assemblie.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=0)
dat_keys = assemblie.datums.keys(); xzdat = assemblie.datums[dat_keys[-1]]
face = shInst.faces.findAt((0,0,bpt/2+shaftl+shelll),)
assemblie.PartitionFaceByDatumPlane(faces=face,datumPlane=xzdat)

#Seed & Mesh
assemblie.seedPartInstance(regions=(shInst,), size=3.0, deviationFactor=0.1,
    minSizeFactor=0.1)
shaftedges = baseInst.edges.getByBoundingBox(xMin=(-shaftr),xMax=1.0,
           yMin=(-shaftr),yMax=shaftr,zMin=bpt,zMax=(shaftl+bpt/2))
assemblie.seedEdgeBySize(edges=shaftedges, size=shaftt, deviationFactor=0.1, 
    minSizeFactor=0.1, constraint=FINER)  
smalledge = baseInst.edges.getByBoundingBox(xMin=(-1.0),xMax=1.0,
           yMin=(-bpr),yMax=bpr,zMin=(bpt/2-iweld),zMax=bpt/2)
assemblie.seedEdgeByNumber(edges=smalledge, number=1, constraint=FINER) 
assemblie.generateMesh(regions=(baseInst,))
assemblie.generateMesh(regions=(shInst,))
                                     
###----------------------------------------------------------------------
#Load End Plate
region1 = shInst.vertices.findAt(((0,0,bpt/2+shaftl+shelll),),)
baseModel.ConcentratedForce(cf2=-1.0, createStepName='Load', 
    localCsys=None, name='Load', region=Region(region1))
    
#Apply load to bolt
assemblie.DatumAxisByPrincipalAxis(principalAxis=ZAXIS)
datkeys = assemblie.datums.keys()
xcen = (arradial/2)*cos(2*pi/num_ar*(.5)+pi/2)
ycen = (arradial/2)*sin(2*pi/num_ar*(.5)+pi/2)
face = arInst.faces.getByBoundingBox(xMin=xcen-ard,xMax=xcen+ard,
               yMin=ycen-ard,yMax=ycen+ard,zMin=bpt/2,zMax=bpt/2)
region = assemblie.Surface(side1Faces=face, name=('BoltSurf0'))
baseModel.BoltLoad(name=('Load0'), createStepName='Bolt',
                       region=region, magnitude=5.0, boltMethod=APPLY_FORCE,
                       datumAxis=assemblie.datums[datkeys[-1]])
face = arInst.faces.getByBoundingBox(xMin=xcen-ard,xMax=xcen+ard,
               yMin=ycen-ard,yMax=ycen+ard,zMin=bpt-5,zMax=bpt-5)
region = regionToolset.Region(faces=face)
baseModel.EncastreBC(name='BC0', createStepName='Initial',
    region=region, localCsys=None)
baseModel.loads['Load0'].setValuesInStep(stepName='Load', 
    boltMethod=FIX_LENGTH)

for i in range(1,num_ar/2):
    xcen = (arradial/2)*cos(2*pi/num_ar*(i+.5)+pi/2)
    ycen = (arradial/2)*sin(2*pi/num_ar*(i+.5)+pi/2)
    face = arInst2[i-1].faces.getByBoundingBox(xMin=xcen-ard,xMax=xcen+ard,
               yMin=ycen-ard,yMax=ycen+ard,zMin=bpt/2,zMax=bpt/2)
    region = assemblie.Surface(side1Faces=face, name=('BoltSurf'+str(i)))
    baseModel.BoltLoad(name=('Load'+str(i)), createStepName=('Bolt'),
                           region=region, magnitude=5.0, boltMethod=APPLY_FORCE,
                           datumAxis=assemblie.datums[datkeys[-1]])
    face = arInst2[i-1].faces.getByBoundingBox(xMin=xcen-ard,xMax=xcen+ard,
               yMin=ycen-ard,yMax=ycen+ard,zMin=bpt-5,zMax=bpt-5)
    region = regionToolset.Region(faces=face)
    baseModel.EncastreBC(name=('BC'+str(i)), createStepName='Initial',
                       region=region, localCsys=None)
    baseModel.loads['Load'+str(i)].setValuesInStep(stepName='Load', 
    boltMethod=FIX_LENGTH)                   


###----------------------------------------------------------------------
#Boundary Conditions
#XSymmetry
symmFaces = baseInst.faces.getByBoundingBox(xMin=-.01,xMax=1.0,
           yMin=(-bpr-.1),yMax=(bpr+.1),zMin=-.1,zMax=(shaftl+shelll+bpt))
symmEdges=shInst.edges.getByBoundingBox(xMin=-.01,xMax=1.0,
           yMin=(-bpr-.1),yMax=(bpr+.1),zMin=-.1,zMax=(shaftl+shelll+bpt))
region1 = assemblie.Surface(side1Edges=symmEdges,name='symmEdges')
region2 = assemblie.Surface(side1Faces=symmFaces,name='symmFaces')
baseModel.XsymmBC(createStepName='Initial', name='Symmetry', region=
    Region(faces=symmFaces, edges=symmEdges))
     
#----------------------------------------------------------------------
#Create field output request
baseModel.fieldOutputRequests.changeKey(
    fromName='F-Output-1', toName='Selected Field Outputs')

baseModel.fieldOutputRequests['Selected Field Outputs'].setValues(
    variables=('S', 'E', 'U', 'RF', 'CF'))

#----------------------------------------------------------------------
#Create history output request

baseModel.HistoryOutputRequest(name='Default History Outputs',
                                createStepName='Load', variables=PRESELECT)
                                
###----------------------------------------------------------------------
###Create and run job
mdb.Job(name='Job1', model='HMIP Base Model', description='', type=ANALYSIS, 
    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
    scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=1, 
    numGPUs=0)
t=0
mdb.jobs['Job1'].submit(consistencyChecking=OFF)


