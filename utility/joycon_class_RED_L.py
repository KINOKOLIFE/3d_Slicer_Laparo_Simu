import vtk, qt, ctk, slicer
import pyjoycon
from pyjoycon import  get_L_id
import time
import numpy as np
from scipy.spatial.transform import Rotation
import quaternion

class Button :
    def __init__(self, name):
        self.name = name
        self.state = False
    def check(self, tag):
        if self.name == tag:
            print("got it", self.name)
    def funcFlip(self, value):
        if value == 0 and self.state:
            print("off")
            self.state = False
        if value == 1 and self.state == False:
            print("on", self.name)
            self.state = True
    def funcFlip(self, tag, value):
        if self.name == tag:
            if value == 0 and self.state:
                print("off")
                self.state = False
            if value == 1 and self.state == False:
                print("on", self.name)
                self.state = True
                
class MyJoyCon(
        pyjoycon.GyroTrackingJoyCon,
        pyjoycon.ButtonEventJoyCon,
    ):
    pass
#---------- main class
class JoyController:
    def __init__(self):
        self.alive = False
        self.joyconTransformNode = slicer.vtkMRMLTransformNode()
        self.joyconTransformNode.SetName('joyconRedTransformation')
        slicer.mrmlScene.AddNode(self.joyconTransformNode)
        self.modelNode = slicer.util.loadModel('/Users/takakiyoshiroshi/Documents/virtual_endoscope_project/3d_model/joyconRed_L/red_parts.obj')
        #self.modelNode = slicer.util.loadModel('./3d_model/joyconRed_L/red_parts.obj')
        self.modelDisplayNode = self.modelNode.GetDisplayNode()
        self.modelDisplayNode.SetColor(0.5, 0.5, 0.5)
        self.modelNode.SetAndObserveTransformNodeID(self.joyconTransformNode.GetID())#0, 195, 227
        modelNode = slicer.util.loadModel('/Users/takakiyoshiroshi/Documents/virtual_endoscope_project/3d_model/joyconRed_L/black_parts.obj')
        modelDisplayNode = modelNode.GetDisplayNode()
        modelDisplayNode.SetColor(0,0,0)
        modelNode.SetAndObserveTransformNodeID(self.joyconTransformNode.GetID())
        modelNode = slicer.util.loadModel('/Users/takakiyoshiroshi/Documents/virtual_endoscope_project/3d_model/click_line/click_line_body.obj')
        modelDisplayNode = modelNode.GetDisplayNode()
        modelDisplayNode.SetColor(0,0,0)
        modelNode.SetAndObserveTransformNodeID(self.joyconTransformNode.GetID())
        modelNode = slicer.util.loadModel('/Users/takakiyoshiroshi/Documents/virtual_endoscope_project/3d_model/click_line/click_line_tip.obj')
        modelDisplayNode = modelNode.GetDisplayNode()
        modelDisplayNode.SetColor(0.8,0.8,0.8)
        modelNode.SetAndObserveTransformNodeID(self.joyconTransformNode.GetID())
        print('joycon connection completed')
        tags = ["buttons/right/y", "buttons/right/y", "buttons/right/a", "buttons/right/b"\
        , "buttons/shared/home", "buttons/shared/plus", \
            "buttons/right/r", "buttons/right/zr", "buttons/right/sr", "buttons/right/sl","buttons/shared/r-stick"]
        self.Buttons = []
        for tag in tags:
                self.Buttons.append(Button(tag))
        self.rbutton = False # joycon ????????????????????????????????????????????????
        self.interState = 0 # ?????????????????????
        self.clutch = False #???????????????
        self.previous = np.identity(4)
        self.rot = np.identity(4)
        self.delta = np.identity(4)
        self.joycon_raw = np.identity(4)
        self.joycon_matrix = np.identity(4) 
        self.zposition = -200
        self.focus = 0 # 0 : camera , 1: forceps
        self.joycon = None
        
    def setup(self):
        self.joycon_id = get_L_id()       
        try:
            self.joycon = MyJoyCon(*self.joycon_id)
        except Exception as e:
            print(e)
            #exit()
            return
        time.sleep(0.5)
        if self.joycon is not None:
            self.joycon.set_player_lamp(0x05)
            self.modelDisplayNode.SetColor(1, 69/255, 84/255)
        
        self.alive = True
        # 255, 69, 84
                
#flatten ?????????
    def dict_flatten(self, d, sep="/"):
        if not isinstance(d, dict):
            raise ValueError
        if not any(filter(lambda x: isinstance(x, dict), d.values())):
            return d

        dct = {}
        for key, value in d.items():
            if isinstance(value, dict):
                for k, v in self.dict_flatten(value, sep).items():
                    dct[key + sep + k] = v
            else:
                dct[key] = value

        return dct
    def update(self, camera_port_ras):
        if not self.alive:
            self.joycon_matrix = np.identity(4)
        else:
            if self.clutch :
                self.joycon_matrix = self.previous.copy()
            if not self.clutch :
                self.joycon_matrix =  np.dot(self.delta , self.joycon_raw)
        
            
        camera_port_matrix = np.identity(4)
        camera_port_matrix[0,3] = camera_port_ras[0]
        camera_port_matrix[1,3] = camera_port_ras[1]
        camera_port_matrix[2,3] = camera_port_ras[2]
        camera_port_matrix = np.dot(camera_port_matrix, self.joycon_matrix)
        transZmat = np.identity(4)
        transZmat[2,3] = self.zposition
       
        self.joycon_matrix = np.dot(camera_port_matrix  , transZmat)
        #print(self.joycon_matrix)
        self.joyconTransformNode.SetMatrixTransformToParent(slicer.util.vtkMatrixFromArray(self.joycon_matrix))
        #self.joycon_matrix : for endoscope object
        #camera_port_matrix : for camera port object
        if self.joycon is not None:
            statusR = self.dict_flatten(self.joycon.get_status())
            x = self.joycon.direction_Q[0]
            y = self.joycon.direction_Q[1]
            z = self.joycon.direction_Q[2]
            w = self.joycon.direction_Q[3]
            quat = np.quaternion(w,x,y,z)   #w, x, y, z
            dcm = quaternion.as_rotation_matrix(quat)
            self.joycon_raw[0:3, 0:3] = dcm
            joycon_mat = np.identity(4)
            
            for k, v in statusR.items():
                for bt in self.Buttons:
                    if k == "buttons/left/up" and v == 1:
                        self.zposition += 0.1
                    if k == "buttons/left/down" and v == 1:
                        self.zposition -= 0.1
                    if k == "buttons/left/l" and v == 1: 
                        if not self.rbutton :
                            self.rbutton = True
                            if self.interState == 0:
                                self.interState = 1
                                self.clutch = True
                                self.previous = np.dot(self.delta, self.joycon_raw)
                                print('clatch!')
                    if k =="buttons/left/zl" and v ==1:
                        print('change')   
                        if self.focus == 0:
                            joycon_raw_inverse = np.linalg.inv(self.joycon_raw)
                            self.delta = np.dot(self.previous, joycon_raw_inverse)
                        self.focus += 1
                        self.focus = self.focus % 2
                                     
                    if k == "buttons/left/l" and v == 0: 
                        if self.rbutton :
                            self.rbutton = False
                            if self.interState == 1:
                                self.interState = 2
                            else:
                                if self.interState == 2:
                                    self.interState = 0
                                    self.clutch = False
                                    print('connect')
                                    joycon_raw_inverse = np.linalg.inv(self.joycon_raw)
                                    self.delta = np.dot(self.previous, joycon_raw_inverse)
        return self.joycon_matrix ,camera_port_matrix
       
       
       
    