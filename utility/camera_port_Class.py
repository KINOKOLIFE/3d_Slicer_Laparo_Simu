from __main__ import vtk, qt, ctk, slicer
import numpy as np

class Camera_Port():
    def __init__(self):
        self.port_position = [0,0,0] #---- initial port position
        markupsNodeID = slicer.modules.markups.logic().AddNewFiducialNode()
        self.fidNode = slicer.util.getNode(markupsNodeID)
        p = self.fidNode.AddFiducial( self.port_position[0],  self.port_position[1],  self.port_position[2])
        self.fidNode.SetNthFiducialLabel(p, "camera port")
        #self.fidNode.AddObserver(slicer.vtkMRMLMarkupsFiducialNode.PointModifiedEvent, self.onPointsModified)
        #-----------
        self.cameraPortTransform = slicer.vtkMRMLTransformNode()
        self.cameraPortTransform.SetName('cameraPortTransformation')
        slicer.mrmlScene.AddNode(self.cameraPortTransform)
        modelNode = slicer.util.loadModel('/Users/takakiyoshiroshi/Documents/virtual_endoscope_project/3d_model/trocars/trocal_large.obj')
        cameraPortDisplayNode = modelNode.GetDisplayNode()
        cameraPortDisplayNode.SetColor(0,0,255)
        modelNode.SetAndObserveTransformNodeID(self.cameraPortTransform.GetID())
        #----------
        cameraPortMatrix = np.identity(4)
        cameraPortMatrix[0,3] = self.port_position[0]
        cameraPortMatrix[1,3] = self.port_position[1]
        cameraPortMatrix[2,3] = self.port_position[2]
        self.offsetZ = 0
        self.update(cameraPortMatrix)
    
    def update(self, camera_port_matrix):
        camera_port_matrix [2,3]  +=  self.offsetZ
        self.cameraPortTransform.SetAndObserveMatrixTransformToParent(slicer.util.vtkMatrixFromArray(camera_port_matrix))
        
    def onPointsModified(self, observer=None, eventid=None):
        return
        ras = vtk.vtkVector3d(0,0,0)
        self.fidNode.GetNthControlPointPosition(0, ras)  
        self.port_position[0] = ras[0]
        self.port_position[1] = ras[1]
        self.port_position[2] = ras[2]
    
    def setPortPosition(self):
        ras = vtk.vtkVector3d(0,0,0)
        self.fidNode.GetNthControlPointPosition(0, ras)  
        self.port_position[0] = ras[0]
        self.port_position[1] = ras[1]
        self.port_position[2] = ras[2]
        print(ras)
        
    def positionUp(self):
        self.offsetZ += 1
        
    def positionDown(self):
        self.offsetZ -= 1
        
class Camera_Port2():
    def __init__(self):
        self.port_position = [100,100,100] #---- initial port position
        markupsNodeID = slicer.modules.markups.logic().AddNewFiducialNode()
        self.fidNode = slicer.util.getNode(markupsNodeID)
        p = self.fidNode.AddFiducial(self.port_position[0],  self.port_position[1],  self.port_position[2])
        self.fidNode.SetNthFiducialLabel(p, "camera port2")
        #self.fidNode.AddObserver(slicer.vtkMRMLMarkupsFiducialNode.PointModifiedEvent, self.onPointsModified)
        #-----------
        self.cameraPortTransform = slicer.vtkMRMLTransformNode()
        self.cameraPortTransform.SetName('cameraPortTransformation2')
        slicer.mrmlScene.AddNode(self.cameraPortTransform)
        modelNode = slicer.util.loadModel('/Users/takakiyoshiroshi/Documents/virtual_endoscope_project/3d_model/trocars/trocar_small.obj')
        cameraPortDisplayNode = modelNode.GetDisplayNode()
        cameraPortDisplayNode.SetColor(0,0.8,0)
        modelNode.SetAndObserveTransformNodeID(self.cameraPortTransform.GetID())
        #----------
        cameraPortMatrix = np.identity(4)
        cameraPortMatrix[0,3] = self.port_position[0]
        cameraPortMatrix[1,3] = self.port_position[1]
        cameraPortMatrix[2,3] = self.port_position[2]
        self.offsetZ = 0
        self.update(cameraPortMatrix)
        
    def update(self, camera_port_matrix):
        camera_port_matrix [2,3]  +=  self.offsetZ
        self.cameraPortTransform.SetAndObserveMatrixTransformToParent(slicer.util.vtkMatrixFromArray(camera_port_matrix))
        
    def onPointsModified(self, observer=None, eventid=None):
        return
        ras = vtk.vtkVector3d(0,0,0)
        self.fidNode.GetNthControlPointPosition(0, ras)  
        self.port_position[0] = self.ras[0]
        self.port_position[1] = self.ras[1]
        self.port_position[2] = self.ras[2]
        
    def setPortPosition(self):
        ras = vtk.vtkVector3d(0,0,0)
        self.fidNode.GetNthControlPointPosition(0, ras)
        self.port_position[0] = ras[0]
        self.port_position[1] = ras[1]
        self.port_position[2] = ras[2]
        print(ras)
        
    def positionUp(self):
        self.offsetZ += 1
        
    def positionDown(self):
        self.offsetZ -= 1