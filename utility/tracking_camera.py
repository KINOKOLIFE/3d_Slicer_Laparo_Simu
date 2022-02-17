import numpy as np
from scipy.spatial.transform import Rotation

class Tracking_Camera:
    def __init__(self):
        self.localMatrix = np.identity(4)
       
    def setInitialPose(self, joycon_matrix, Camera_2):
        localMatrix = self.getMatrix(joycon_matrix)
        worldMatrix = np.linalg.inv(localMatrix)
        world_cameraPosition = np.append(Camera_2.GetPosition(), [1])
        world_cameraFocalPoint = np.append(Camera_2.GetFocalPoint(), [1])
        self.localCameraPosition = np.dot(worldMatrix, world_cameraPosition)
        self.localFocalPoint = np.dot(worldMatrix, world_cameraFocalPoint)
        
    def update(self, joycon_matrix, Camera_2):
        localMatrix = self.getMatrix(joycon_matrix)
        CameraPosition = np.dot(localMatrix, self.localCameraPosition)
        CameraFocalPoint = np.dot(localMatrix, self.localFocalPoint)
        self.CameraPosition = [CameraPosition[i] for i in (0,1,2)]
        self.CameraFocalPoint = [CameraFocalPoint[i] for i in (0,1,2)]
        zup = np.array([0,0,1])
        Camera_2.SetFocalPoint(self.CameraFocalPoint)
        Camera_2.SetViewUp(zup)
        Camera_2.SetPosition(self.CameraPosition)
        
    def getMatrix(self, joycon):
        joycon_ = np.identity(4)
        joycon_[0:3, 0:3] = joycon[0:3, 0:3]
        axis = np.array([0,0,1,1])
        
        xy = np.dot(joycon_, axis)
        xy[2] = 0 
        xaxis = [xy[i] for i in (0,1,2)]
        
        if np.linalg.norm(xaxis) == 0 : #ジンバルロック
            return joycon
        else:
            xaxis = xaxis / np.linalg.norm(xaxis)
            rotv = Rotation.from_rotvec(np.array([0, 0, np.pi/2]))
            rot = rotv.as_dcm()
            rot4 = np.identity(4)
            rot4[0:3, 0:3] = rot[0:3, 0:3]
            yaxis = np.dot(rot4, xy )
            yaxis = [yaxis[i] for i in (0,1,2)]
            yaxis = yaxis / np.linalg.norm(yaxis)
            zaxis = np.array([0,0,1])
            rot3 = np.array([xaxis , yaxis , zaxis])
            rot3t = rot3.T
            rot44 = np.identity(4)
            rot44[0:3, 0:3] = rot3t[0:3, 0:3]
            rot44[0,3] = joycon[0,3]
            rot44[1,3] = joycon[1,3]
            rot44[2,3] = joycon[2,3]
            return rot44    