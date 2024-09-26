import pybullet as p
import time
import pybullet_data
import os
import numpy as np

import trimesh

# filename, position = 'models/angle_block.STL', [1.05,-0.925,0]
# filename, position = 'models/Duck.glb', [1.05,-0.925,0]
# filename, position = 'models/cylinder.stl', [2,0,0]
# filename, position = 'models/idler_riser.STL', [2.63,-0.75,0]
# filename, position = 'models/unit_sphere.STL', [2,0,0]
# filename, position = 'models/torus.STL', [3,0,0]
filename, position = 'models/unit_cube.STL', [1,0,0]
# filename, position = os.path.join(pybullet_data.getDataPath(), "duck.obj"), [1,.75,0]

def write_convex_decomposition(name_in,name_out):
    if not os.path.isfile(name_out):
        name_log = "log.txt"
        p.vhacd(name_in, name_out, name_log)

def get_convex_decomposed_file(filename):
    mesh = trimesh.load_mesh(filename)
    fn,_ = os.path.splitext(filename)
    name_in = fn + '.obj'
    mesh.export(name_in)
    name_out = fn + '_vhacd.obj'
    write_convex_decomposition(name_in,name_out)
    return name_out

name_out = get_convex_decomposed_file(filename)

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

if p.getConnectionInfo()['connectionMethod']==1:
    # p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)


# box1 = p.createCollisionShape(p.GEOM_MESH,fileName=name_in)#,flags=p.GEOM_FORCE_CONCAVE_TRIMESH) # bullet does not do static concave mesh-mesh collisions
# box2 = p.createCollisionShape(p.GEOM_MESH,fileName=name_in)#,flags=p.GEOM_FORCE_CONCAVE_TRIMESH)

b1 = p.createCollisionShape(p.GEOM_MESH,fileName=name_out)
b2 = p.createCollisionShape(p.GEOM_MESH,fileName=name_out)


object1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=b1)
object2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=b2, basePosition=position)


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
        # print('ptA = ', ["{0:0.2f}".format(i) for i in ptA])
        # print('ptB = ', ["{0:0.2f}".format(i) for i in ptB])
        print('normal = ', ["{0:0.2f}".format(i) for i in normal]) # from B to A
        if p.getConnectionInfo()['connectionMethod']==1:
            p.addUserDebugLine(lineFromXYZ=ptA,
                                lineToXYZ=ptB,
                                lineColorRGB=colorRGB,
                                lineWidth=lineWidth,
                                lifeTime=0)

if p.getConnectionInfo()['connectionMethod']==1: # sleep when GUI 
    time.sleep(30)
p.disconnect()

