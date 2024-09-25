import pybullet as p
import time
import pybullet_data
import os


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

if p.getConnectionInfo()['connectionMethod']==1: # sleep when GUI 
    # p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

name_in = os.path.join(pybullet_data.getDataPath(), "duck.obj")

# box1 = p.createCollisionShape(p.GEOM_BOX,halfExtents = [1,1,1])
# box2 = p.createCollisionShape(p.GEOM_BOX,halfExtents = [1,1,1])
name_out = "duck_vhacd.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log)

# box1 = p.createCollisionShape(p.GEOM_MESH,fileName=name_in)#,flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
# box2 = p.createCollisionShape(p.GEOM_MESH,fileName=name_in)#,flags=p.GEOM_FORCE_CONCAVE_TRIMESH)

box1 = p.createCollisionShape(p.GEOM_MESH,fileName=name_out)
box2 = p.createCollisionShape(p.GEOM_MESH,fileName=name_out)


object1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box1)
object2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box2,
                        basePosition=[1,.75,0])


lineWidth = 3
colorRGB = [0, 0, 0]


contacts = p.getClosestPoints(bodyA=object1, bodyB=object2, distance=0.1) #tolerance
if len(contacts) > 0:
    print('# contacts = ', len(contacts))
    for contact in contacts:
        distance = contact[8]
        ptA = contact[5] # points at the shape boundary along the normal 
        ptB = contact[6]
        normal = contact[7] 
        print("distance = ", "{:.2f}".format(distance))
        print('ptA = ', ["{0:0.2f}".format(i) for i in ptA])
        print('ptB = ', ["{0:0.2f}".format(i) for i in ptB])
        print('normal = ', ["{0:0.2f}".format(i) for i in normal])
        if p.getConnectionInfo()['connectionMethod']==1:
            p.addUserDebugLine(lineFromXYZ=ptA,
                                lineToXYZ=ptB,
                                lineColorRGB=colorRGB,
                                lineWidth=lineWidth,
                                lifeTime=0)

if p.getConnectionInfo()['connectionMethod']==1: # sleep when GUI 
    time.sleep(15)
p.disconnect()