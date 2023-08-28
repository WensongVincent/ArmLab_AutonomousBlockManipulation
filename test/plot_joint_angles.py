import sys
import os
script_path = os.path.dirname(os.path.realpath(__file__))
os.sys.path.append(os.path.realpath(script_path + '/../'))
from kinematics import *
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# Home configuration of the end-effector
M_matrix = np.array([[1,0,0,0],
                        [0,1,0,408.575],
                        [0,0,1,304.57],
                        [0,0,0,1]])
# Joint screw axes in the space frame  
S_list = np.array([[0,0,1,   0,0,0],
            [-1,0,0,  0,-104.57,0],
            [1,0,0,   0,304.57,-50],
            [1,0,0,   0,304.57,-250],
            [0,1,0,   -304.47,0,0]]).T

joint_angles = [[-0.03374757990241051, -0.03374757990241051, -0.08590292930603027, -0.14419420063495636, -0.030679617077112198, 0.7378447651863098, 0.031067059260919835, -0.031067059260919835],
[0.21322333812713623, -0.05368933081626892, -0.2730485796928406, -1.2977477312088013, 0.07976700365543365, 0.7378447651863098, 0.031067059260919835, -0.031067059260919835],
[0.38656318187713623, 0.07363107800483704, -0.36968937516212463, -1.115204095840454, 0.3942330777645111, 0.7378447651863098, 0.031067059260919835, -0.031067059260919835],
[0.3880971372127533, 0.08130098134279251, -0.44025251269340515, -1.0906603336334229, 0.41724279522895813, 0.03374757990241051, 0.019971687660444486, -0.019971687660444486],
[0.3834952116012573, -0.02761165425181389, -0.11658254265785217, -1.512505054473877, 0.41724279522895813, 0.03374757990241051, 0.019971687660444486, -0.019971687660444486],
[0.004601942375302315, -0.030679617077112198, -0.1902136206626892, -1.4496119022369385, 0.07976700365543365, 0.03374757990241051, 0.019971687660444486, -0.01997168766044448],
[-0.42031073570251465, 0.012271846644580364, -0.2899223864078522, -1.3054176568984985, -0.2991262674331665, 0.03374757990241051, 0.019971687660444486, -0.019971687660444486],
[-0.43411657214164734, 0.06596117466688156, -0.4586602747440338, -1.0293011665344238, -0.37275734543800354, 0.03374757990241051, 0.019971687660444486, -0.019971687660444486],
[-0.42951464653015137, 0.052155349403619766, -0.4632622003555298, -1.0354371070861816, -0.37275734543800354, 0.9081166386604309, 0.033438012576536155, -0.033438012576536155],
[-0.23009712994098663, -0.21322333812713623, -0.29452431201934814, -1.24252450466156, -0.22856314480304718, 0.9081166386604309, 0.033438012576536155, -0.033438012576536155]]
x = []
y = []
z = []
for angle in joint_angles:
    pose = FK_pox(angle[:5], M_matrix, S_list)
    xyz = get_pose_from_T(pose)
    xyz_points = xyz[:3]
    x.append(xyz_points[0])
    y.append(xyz_points[1])
    z.append(xyz_points[2])

fig = plt.figure()
j0 = [angle[0] for angle in joint_angles]
j1 = [angle[1] for angle in joint_angles]
j2 = [angle[2] for angle in joint_angles]
j3 = [angle[3] for angle in joint_angles]
j4 = [angle[4] for angle in joint_angles]
j5 = [angle[5] for angle in joint_angles]
# j6 = [angle[6] for angle in joint_angles]
# j7 = [angle[7] for angle in joint_angles]
# j3 = [angle[8] for angle in joint_angles]
# j4 = [angle[4] for angle in joint_angles]
x = range(len(j0))
# j5 = [angle[5] for angle in joint_angles2]
plt.plot(x, j0)
plt.plot(x, j1)
plt.plot(x, j2)
plt.plot(x, j3)
plt.plot(x, j4)
plt.plot(x, j5)
# plt.plot(x, j6)
# plt.plot(x, j7)
# plt.scatter(x, j0)
# plt.scatter(x, j1)
# plt.scatter(x, j2)
# plt.scatter(x, j3)
# plt.scatter(x, j4)
# plt.plot(j5)
plt.xlabel('Waypoint number')
plt.ylabel('Joint angle (rad)')
plt.legend(['Base', 'Shoulder', 'Elbow', 'Wrist Angle', 'Wrist Rotate', 'Gripper'])
plt.show()

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.plot3D(x, y, z)
# ax.scatter3D(x, y, z, color='black')
# ax.set_xlabel('x (mm)')
# ax.set_ylabel('y (mm)')
# ax.set_zlabel('z (mm)')
# plt.show()

joint_angles2 = [[-1.555456519126892, 0.30526217818260193, 0.19941750168800354, -0.15033012628555298, -0.0015339808305725455, -0.47860202193260193, 0.014084647016798079, -0.014084647016798079], 
[-1.1029322147369385, 0.22856314480304718, 0.19788353145122528, -0.15493206679821014, -0.0015339808305725455, -0.47860202193260193, 0.014084647016798079, -0.014084647016798079], 
[-0.6289321184158325, 0.21015536785125732, 0.19941750168800354, -0.15033012628555298, -0.0015339808305725455, -0.47860202193260193, 0.014084647016798079, -0.014084647016798079], 
[-0.17487381398677826, 0.21015536785125732, 0.19941750168800354, -0.15493206679821014, -0.0015339808305725455, -0.47860202193260193, 0.014084647016798079, -0.014084647016798079], 
[0.3267379105091095, 0.2208932340145111, 0.19941750168800354, -0.15186409652233124, -0.0015339808305725455, -0.47860202193260193, 0.014084647016798079, -0.014084647016798079], 
[0.7010292410850525, 0.22242721915245056, 0.19941750168800354, -0.15493206679821014, -0.0015339808305725455, -0.47860202193260193, 0.014084647016798079, -0.014084647016798079], 
[1.1198060512542725, 0.2485048919916153, 0.19941750168800354, -0.1564660519361496, -0.0015339808305725455, -0.47860202193260193, 0.014084647016798079, -0.014084647016798079], 
[1.431204080581665, 0.25003886222839355, 0.19941750168800354, -0.15493206679821014, -0.0015339808305725455, -0.47860202193260193, 0.014084647016798079, -0.014084647016798079], 
[1.5830682516098022, 0.26384469866752625, 0.21629129350185394, -0.1564660519361496, -0.0015339808305725455, -0.47860202193260193, 0.014084647016798079, -0.014084647016798079]]
x = []
y = []
z = []
for angle in joint_angles2:
    pose = FK_pox(angle[:5], M_matrix, S_list)
    xyz = get_pose_from_T(pose)
    xyz_points = xyz[:3]
    x.append(xyz_points[0])
    y.append(xyz_points[1])
    z.append(xyz_points[2])

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.plot3D(x, y, z)
# ax.scatter3D(x, y, z, color='black')
# ax.set_xlabel('x (mm)')
# ax.set_ylabel('y (mm)')
# ax.set_zlabel('z (mm)')
# plt.show()

fig = plt.figure()
j0 = [angle[0] for angle in joint_angles2]
j1 = [angle[1] for angle in joint_angles2]
j2 = [angle[2] for angle in joint_angles2]
j3 = [angle[3] for angle in joint_angles2]
j4 = [angle[4] for angle in joint_angles2]
j5 = [angle[4] for angle in joint_angles2]
x = range(len(j0))
# j5 = [angle[5] for angle in joint_angles2]
plt.plot(x, j0)
plt.plot(x, j1)
plt.plot(x, j2)
plt.plot(x, j3)
plt.plot(x, j4)
# plt.scatter(x, j0)
# plt.scatter(x, j1)
# plt.scatter(x, j2)
# plt.scatter(x, j3)
# plt.scatter(x, j4)
plt.plot(j5)
# plt.scatter(x, j5)
plt.xlabel('Waypoint number')
plt.ylabel('Joint angle (rad)')
plt.legend(['Base', 'Shoulder', 'Elbow', 'Wrist Angle', 'Wrist Rotate', 'Gripper'])
plt.show()


# plt.plot[joint_angles[0][:])
# plt.plot(joint_angles[1][:])
# plt.plot(joint_angles[2][:])
# plt.plot(joint_angles[3][:])
# plt.plot(joint_angles[4][:])
# plt.plot(joint_angles[5][:])
# plt.plot(joint_angles[6][:])
# plt.plot(joint_angles[7][:])
# plt.ylabel('some numbers')
# 
