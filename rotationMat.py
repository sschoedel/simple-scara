import numpy as np

np.set_printoptions(precision=5)

def rotateXYZ(pos, thetas, rads=True):

    if not rads:
        thetas = [thetas[i]*np.pi/180 for i in range(len(thetas))]

    # rotation about x
    X = np.matrix([[1, 0, 0],
                   [0, np.cos(thetas[0]), -np.sin(thetas[0])],
                   [0, np.sin(thetas[0]), np.cos(thetas[0])]])

    # rotation about y
    Y = np.matrix([[np.cos(thetas[1]), 0, np.sin(thetas[1])],
                   [0, 1, 0],
                   [-np.sin(thetas[1]), 0, np.cos(thetas[1])]])

    # rotation about z
    Z = np.matrix([[np.cos(thetas[2]), -np.sin(thetas[2]), 0],
                   [np.sin(thetas[2]), np.cos(thetas[2]), 0],
                   [0, 0, 1]])

    return X*Y*Z*pos

print(rotateXYZ([[1],[0],[0]],[0,0,90], rads=False))



def ZTransformMatrix(theta, startLengths, rads=True):
    if not rads:
        theta = theta * np.pi/180

    return np.array([[np.cos(theta), -np.sin(theta), 0, startLengths[0]],
                    [np.sin(theta), np.cos(theta), 0, startLengths[1]],
                    [0, 0, 1, startLengths[2]],
                    [0, 0, 0, 1]])

theta1 = 45
start1 = np.array([0,0,0])
theta2 = 0
start2 = np.array([1,0,0])
theta3 = 0
start3 = np.array([1,0,0])


mat1 = ZTransformMatrix(theta1, start1, rads=False)
mat2 = ZTransformMatrix(theta2, start2, rads=False)
mat3 = ZTransformMatrix(theta3, start3, rads=False)

transform01 = mat1
transform02 = mat1 @ mat2
transform03 = transform02 @ mat3

baseTransforms = np.array([transform01, transform02, transform03])

# Extract position and rotation data for each joint
jointPositions = np.array([baseTransforms[i,:-1,3] for i in range(0, baseTransforms.shape[0])])
print(jointPositions)
