import vtk, qt, ctk, slicer
import time
import math
import numpy as np

class Endoscope:
    def __init__(self, wedget):
        pass
    def update(self, joycon_mat, camera):
        pass
    def hide(self):
        pass
    def show(self):
        pass
    def getZRotMa(self, degree):
        thea =  math.radians(degree)
        c = math.cos(thea)
        s = math.sin(thea)
        mat = np.array([    [   c,      -s,     0.0,    0.0],
                            [   s,      c,      0.0,    0.0],
                            [   0.0,   0.0,     1.0,    0.0],
                            [   0.0,   0.0,     0.0,    1.0] ]    )
        return mat
    def getYRotMat(self, degree):
        thea = math.radians(degree)
        c = math.cos(thea)
        s = math.sin(thea)
        mat = np.array([    [c,     0.0,    s,      0.0],
                            [0.0,   1.0,    0.0,    0.0],
                            [-s,    0.0,    c,      0.0],
                            [0.0,   0.0,    0.0,    1.0] ]    )
        return mat 
    def getXRotMat(self, degree):
        thea = math.radians(degree)
        c = math.cos(thea)
        s = math.sin(thea)
        mat = np.array([    [1.0,     0.0,    0.0,      0.0],
                            [0.0,   c,    -s,    0.0],
                            [0.0,    s,    c,      0.0],
                            [0.0,   0.0,    0.0,    1.0] ]    )
        return mat         
        
    
class Hopkins0(Endoscope):
    def __init__(self):
        self.scopeTransform = slicer.vtkMRMLTransformNode()
        self.scopeTransform.SetName('scopeTransformation_h0')
        slicer.mrmlScene.AddNode(self.scopeTransform)
        modelNode = slicer.util.loadModel('/Users/takakiyoshiroshi/Desktop/slicer3d_model/hopkins0/parts.obj')
        self.modelDisplayNode = modelNode.GetDisplayNode()
        self.modelDisplayNode.SetColor(201/255, 201/255, 202/255)
        modelNode.SetAndObserveTransformNodeID(self.scopeTransform.GetID())#0, 195, 227
        self.service = True
  
    
    def hide(self):
        self.modelDisplayNode.SetVisibility(False)
    
    def show(self):
        self.modelDisplayNode.SetVisibility(True)
    def startService(self):
        self.service = True
    def stopService(self):
        self.service = False
        
    def update(self, joycon_mat, camera, s):
        if self.service:
            joycon_to_scopeBase = np.identity(4)
            joycon_to_scopeBase[2,3] = 60
            scopeBase = np.dot(joycon_mat , joycon_to_scopeBase)
            self.scopeTransform.SetAndObserveMatrixTransformToParent(slicer.util.vtkMatrixFromArray(scopeBase))
            #--------
            rotZ90p = self.getZRotMa(90)
            joy_scopetip = np.identity(4)
            joy_scopetip[2,3] = 500
            cam_mat = np.dot(joycon_mat , joy_scopetip)
            cam_mat = np.dot(cam_mat, rotZ90p)
            cameraPosition = [ cam_mat[0,3] , cam_mat[1,3] , cam_mat[2,3] ]
            ez = np.array([0.0,0.0,100.0,1.0])
            focalPoint =  cam_mat@ez
            focalPoint = [focalPoint[i] for i in (0,1,2)]
            camera.SetFocalPoint(focalPoint)
            cam_mat[0,3] = 0
            cam_mat[1,3] = 0
            cam_mat[2,3] = 0
            ey = np.array([0.0,1.0,0.0,1.0])
            Zup = cam_mat@ey
            Zup = [Zup[i] for i in (0,1,2)]
            camera.SetViewUp(Zup)
            camera.SetPosition(cameraPosition)

class Hopkins30(Endoscope):
    pass
'''    
class Hopkins30(Endoscope):
    def __init__(self, wedget):
        modelNode1 = slicer.util.loadModel('/Users/takakiyoshiroshi/Desktop/slicer3d_model/hopkins/silver_parts.obj')
        self.modelDisplayNode1 = modelNode1.GetDisplayNode()
        self.modelDisplayNode1.SetColor(201, 201, 202)
        modelNode1.SetAndObserveTransformNodeID(wedget.scopeTransform.GetID())#0, 195, 227
        modelNode2 = slicer.util.loadModel('/Users/takakiyoshiroshi/Desktop/slicer3d_model/hopkins/black_parts.obj')
        self.modelDisplayNode2 = modelNode2.GetDisplayNode()
        self.modelDisplayNode2.SetColor(0,0,0)
        modelNode2.SetAndObserveTransformNodeID(wedget.scopeTransform.GetID())
        self.hide()
    
    def hide(self):
        self.modelDisplayNode1.SetVisibility(False)
        self.modelDisplayNode2.SetVisibility(False)
    
    def show(self):
        self.modelDisplayNode1.SetVisibility(True)
        self.modelDisplayNode2.SetVisibility(True)
        
    def update(self, wedget):
        joycon_to_scopeBase = np.identity(4)
        joycon_to_scopeBase[2,3] = 60
        scopeBase = np.dot(wedget.joycon_mat , joycon_to_scopeBase)
        rotBase = self.getZRotMa(wedget.cameraDegree)
        scopeBase = np.dot(scopeBase , rotBase)
        wedget.scopeTransform.SetAndObserveMatrixTransformToParent(slicer.util.vtkMatrixFromArray(scopeBase))
        #---
        rotBase_ = self.getZRotMa( - wedget.cameraDegree )
        rotZ90p = self.getZRotMa(90)
        joy_scopetip = np.identity(4)
        joy_scopetip[2,3] = 500
        cam_mat = np.dot(wedget.joycon_mat , joy_scopetip)
        cam_mat = np.dot(cam_mat, rotBase)
        rotY30 = self.getYRotMat(30)
        cam_mat = np.dot(cam_mat, rotY30)
        cam_mat = np.dot(cam_mat, rotBase_)
        cam_mat = np.dot(cam_mat, rotZ90p)
        cameraPosition = [ cam_mat[0,3] , cam_mat[1,3] , cam_mat[2,3] ]
        ez = np.array([0.0,0.0,100.0,1.0])
        focalPoint =  cam_mat@ez
        focalPoint = [focalPoint[i] for i in (0,1,2)]
        wedget.camera.SetFocalPoint(focalPoint)
        cam_mat[0,3] = 0
        cam_mat[1,3] = 0
        cam_mat[2,3] = 0
        ey = np.array([0.0,1.0,0.0,1.0])
        Zup = cam_mat@ey
        Zup = [Zup[i] for i in (0,1,2)]
        wedget.camera.SetViewUp(Zup)
        wedget.camera.SetPosition(cameraPosition)

class OlympusFrexible(Endoscope):
    def __init__(self, wedget):
        modelNode1 = slicer.util.loadModel('/Users/takakiyoshiroshi/Desktop/slicer3d_model/visera/silver_parts.obj')
        self.modelDisplayNode1 = modelNode1.GetDisplayNode()
        self.modelDisplayNode1.SetColor(201, 201, 202)
        modelNode1.SetAndObserveTransformNodeID(wedget.scopeTransform.GetID())#0, 195, 227
        modelNode2 = slicer.util.loadModel('/Users/takakiyoshiroshi/Desktop/slicer3d_model/visera/black_parts.obj')
        self.modelDisplayNode2 = modelNode2.GetDisplayNode()
        self.modelDisplayNode2.SetColor(0,0,0)
        modelNode2.SetAndObserveTransformNodeID(wedget.scopeTransform.GetID())
        modelNode3 = slicer.util.loadModel('/Users/takakiyoshiroshi/Desktop/slicer3d_model/visera/top_silver_parts.obj')
        self.modelDisplayNode3 = modelNode3.GetDisplayNode()
        self.modelDisplayNode3.SetColor(201, 201, 202)
        modelNode3.SetAndObserveTransformNodeID(wedget.scopeTipTransform.GetID())#0, 195, 227
        modelNode4 = slicer.util.loadModel('/Users/takakiyoshiroshi/Desktop/slicer3d_model/visera/top_black_parts.obj')
        self.modelDisplayNode4 = modelNode4.GetDisplayNode()
        self.modelDisplayNode4.SetColor(0,0,0)
        modelNode4.SetAndObserveTransformNodeID(wedget.scopeTipTransform.GetID())
        self.hide()
    
    def hide(self):
        self.modelDisplayNode1.SetVisibility(False)
        self.modelDisplayNode2.SetVisibility(False)
        self.modelDisplayNode3.SetVisibility(False)
        self.modelDisplayNode4.SetVisibility(False)
    
    def show(self):
        self.modelDisplayNode1.SetVisibility(True)
        self.modelDisplayNode2.SetVisibility(True)
        self.modelDisplayNode3.SetVisibility(True)
        self.modelDisplayNode4.SetVisibility(True)
        
    def update(self, wedget):
        #364mm
        joycon_to_scopeBase = np.identity(4)
        joycon_to_scopeBase[2,3] = 60
        scopeBase = np.dot(wedget.joycon_mat , joycon_to_scopeBase)
        wedget.scopeTransform.SetAndObserveMatrixTransformToParent(slicer.util.vtkMatrixFromArray(scopeBase))
        scopeBase_to_scopeTip = np.identity(4)
        scopeBase_to_scopeTip[2,3] = 364
        scopeTip = np.dot(scopeBase , scopeBase_to_scopeTip)
        roty = self.getYRotMat(-wedget.cameraRotY)
        scopeTip = np.dot(scopeTip , roty)
        rotx = self.getXRotMat(-wedget.cameraRotX)
        scopeTip = np.dot(scopeTip , rotx)
        
        wedget.scopeTipTransform.SetAndObserveMatrixTransformToParent(slicer.util.vtkMatrixFromArray(scopeTip))
        #---
        scopeTip_to_cam = np.identity(4)
        scopeTip_to_cam[2,3] = 60
        scopeTip = np.dot(scopeTip , scopeTip_to_cam)
        rotZ90p = self.getZRotMa(90)
        cam_mat = np.dot(scopeTip, rotZ90p)
        cameraPosition = [ cam_mat[0,3] , cam_mat[1,3] , cam_mat[2,3] ]
        ez = np.array([0.0,0.0,100.0,1.0])
        focalPoint =  cam_mat@ez
        focalPoint = [focalPoint[i] for i in (0,1,2)]
        wedget.camera.SetFocalPoint(focalPoint)
        cam_mat[0,3] = 0
        cam_mat[1,3] = 0
        cam_mat[2,3] = 0
        ey = np.array([0.0,1.0,0.0,1.0])
        Zup = cam_mat@ey
        Zup = [Zup[i] for i in (0,1,2)]
        wedget.camera.SetViewUp(Zup)
        wedget.camera.SetPosition(cameraPosition)
        '''