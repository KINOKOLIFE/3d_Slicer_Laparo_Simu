from __main__ import vtk, qt, ctk, slicer
from slicer.ScriptedLoadableModule import *
import numpy as np
import quaternion
from scipy.spatial.transform import Rotation
from utility import endoscope_class
from utility import joycon_class
from utility import tracking_camera
from utility import joycon_class_RED_L
from utility import camera_port_Class     
    
class virtual_scope(ScriptedLoadableModule):
    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        parent.title = "VR_scope"
        parent.categories = ["Virtual Endoscope"]
        parent.dependencies = []
        parent.contributors = ["yocchan"] 
        parent.helpText = """
        Example of scripted loadable extension for the HelloPython tutorial.
        """
        parent.acknowledgementText = """
        """ 
        self.parent = parent

class virtual_scopeWidget(ScriptedLoadableModuleWidget):
    def __init__(self, parent = None):
        ScriptedLoadableModuleWidget.__init__(self, parent)
        if not parent:
            self.parent = slicer.qMRMLWidget()
            self.parent.setLayout(qt.QVBoxLayout())
            self.parent.setMRMLScene(slicer.mrmlScene)
        else:
            self.parent = parent
            self.layout = self.parent.layout()
        if not parent:
            self.setup()
            self.parent.show()

    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)
                                                #-------ここまでデフォルト
    #------
        self.joyController = joycon_class.JoyController()
        self.joyController_RED_L = joycon_class_RED_L.JoyController()
        self.mat = np.identity(4)
        
    #-----手元カメラ
        self.tracking_camera = tracking_camera.Tracking_Camera()
        self.camera_2Node = slicer.util.getNode('vtkMRMLCameraNode2')
        self.camera_2 = self.camera_2Node.GetCamera()
        self.camera_2.SetPosition([200,200,300])
        self.camera_2.SetFocalPoint([0,0,0])
        self.camera_2.SetViewUp([0,0,1])
        self.camera_2.AddObserver(vtk.vtkCommand.ModifiedEvent, self.onCamera_2NodeModified)
        ras = [0,0,0]
        mat, o ,s = self.joyController.update(ras)
        self.tracking_camera.setInitialPose(self.mat, self.camera_2)
        self.camera_2Node.ResetClippingRange()
    #----endo scope
        self.cameraNode = slicer.util.getNode('vtkMRMLCameraNode1')
        self.camera = self.cameraNode.GetCamera()
        self.cameraNode.ResetClippingRange()
        
        Hopkins0 = endoscope_class.Hopkins0()
        Hopkins30 = endoscope_class.Hopkins30()
        Flexible = endoscope_class.Frexible()
        
        self.deviceList = [Hopkins0, Hopkins30, Flexible]
        self.deviceIndex = 0
        
    #----perspective
        self.camera_3Node = slicer.util.getNode('vtkMRMLCameraNode3')
        self.camera_3Node.ResetClippingRange()
    #--------camera port
        self.camera_port = camera_port_Class.Camera_Port()
        self.instrument_port = camera_port_Class.Camera_Port2()
        
    #-------- update every 20msec
        self.timer = qt.QTimer()
        self.timer.setInterval(20)
        self.timer.connect('timeout()', self.update)
        self.timer.start()
        
    # GUI
        self.draw_gui()

    def update(self):
        self.mat, camera_port_matrix, s = self.joyController.update(self.camera_port.port_position)
        self.camera_port.update(camera_port_matrix)  
        #self.tracking_camera.update(self.mat, self.camera_2)
        self.deviceList[self.deviceIndex].update(self.mat, self.camera, s)
        
        mat1, mat2  = self.joyController_RED_L.update(self.instrument_port.port_position)
        self.instrument_port.update(mat2)  
        
        self.cameraNode.ResetClippingRange()
        
        
    def onCamera_2NodeModified(self, observer, eventid):
        self.tracking_camera.setInitialPose(self.mat, self.camera_2)
    
    def draw_gui(self):
        # Collapsible button
        sampleCollapsibleButton = ctk.ctkCollapsibleButton()
        sampleCollapsibleButton.text = "virtual endoscope"
        self.layout.addWidget(sampleCollapsibleButton)
        # Layout within the sample collapsible button
        sampleFormLayout = qt.QFormLayout(sampleCollapsibleButton)
        #-----------ここまでデフォルト
        #ラベル
        label1 = qt.QLabel('connectする前にbluetooth接続を確認')
        sampleFormLayout.addRow('注意', label1)
        #joycon connection
        raw1_layout = qt.QGridLayout()
        button_R_BLUE = qt.QPushButton("connect Blue(R)")
        button_R_BLUE.setCheckable(True)
        button_R_BLUE.setFixedSize(120, 20)
        button_L_RED = qt.QPushButton("connect RED(L)")
        button_L_RED.setCheckable(True)
        #button_L_RED.setChecked(True)
        button_L_RED.setFixedSize(120, 20)
        raw1_layout.addWidget(button_R_BLUE, 0, 0)
        raw1_layout.addWidget(button_L_RED, 0, 1)
        self.button_R_BLUE = button_R_BLUE
        self.button_L_RED = button_L_RED
        button_R_BLUE.toggled.connect(self.button_R_BLUE_toggled)
        button_L_RED.toggled.connect(self.button_L_RED_toggled)
        sampleFormLayout.addRow("joycon: ", raw1_layout )   
        #----カメラポート
        horizen_layout1 = qt.QHBoxLayout()
        button_port_setter = qt.QPushButton("set") 
        button_port_setter.clicked.connect(self.button_port_setter_toggled)
        button_port_up = qt.QPushButton("up") 
        button_port_up.clicked.connect(self.button_port_up_toggled)
        button_port_down = qt.QPushButton("down") 
        button_port_down.clicked.connect(self.button_port_down_toggled)
        horizen_layout1.addWidget(button_port_setter)
        horizen_layout1.addWidget(button_port_up)
        horizen_layout1.addWidget(button_port_down)
        sampleFormLayout.addRow("camera port : ", horizen_layout1 ) 
        #----インスツルメントポート
        horizen_layout2 = qt.QHBoxLayout()
        button_instrument_setter = qt.QPushButton("set") 
        button_instrument_setter.clicked.connect(self.button_instrument_setter_toggled)
        button_instrument_up = qt.QPushButton("up") 
        button_instrument_up.clicked.connect(self.button_instrument_up_toggled)
        button_instrument_down = qt.QPushButton("down") 
        button_instrument_down.clicked.connect(self.button_instrument_down_toggled)
        horizen_layout2.addWidget(button_instrument_setter)
        horizen_layout2.addWidget(button_instrument_up)
        horizen_layout2.addWidget(button_instrument_down)
        sampleFormLayout.addRow("instrument port : ", horizen_layout2 ) 
        # skin nodeの選択    
        seglayout = qt.QHBoxLayout()
        segmentSelector = slicer.qMRMLNodeComboBox()
        segmentSelector.objectName = 'segmentationSelector'
        segmentSelector.toolTip = "view2 skin segment"
        segmentSelector.nodeTypes = ['vtkMRMLSegmentationNode']#  vtkMRMLSegmentationNode
        segmentSelector.noneEnabled = False
        segmentSelector.addEnabled = False
        segmentSelector.removeEnabled = False
        
        seglayout.addWidget(segmentSelector)
        
        self.button1 = qt.QPushButton('hide')
        self.button1.setCheckable(True)
        self.button1.toggled.connect(self.view2_visble_button_toggled)
        seglayout.addWidget(self.button1)
        sampleFormLayout.addRow("view2 skin segment:", seglayout)
        segmentSelector.setMRMLScene(slicer.mrmlScene)
        segmentSelector.connect('currentNodeChanged(vtkMRMLNode*)', self.selectSkinNode)
        
        #--select scope type
        combo = qt.QComboBox()
        combo = qt.QComboBox()
        combo.addItem('10mm Rigid 0deg')
        combo.addItem('10mm Rigid 30deg')
        combo.addItem('10mm flexible')
        sampleFormLayout.addRow("scope type:", combo)
        combo.currentIndexChanged.connect(self.device_changed)
        self.device_changed(0)
    
    def device_changed(self, index):
        for device in self.deviceList:
            device.hide()
        self.deviceIndex = index
        self.deviceList[self.deviceIndex].show()

    def button_R_BLUE_toggled(self):
        self.button_R_BLUE.setEnabled(False)
        self.joyController.setup()
      
    def button_L_RED_toggled(self):
        self.button_L_RED.setEnabled(False)
        self.joyController_RED_L.setup()
    
    def button_port_setter_toggled(self):
        self.camera_port.setPortPosition()
        
    def button_port_up_toggled(self):
        self.camera_port.positionUp()
        
    def button_port_down_toggled(self):
        self.camera_port.positionDown()
        
    def button_instrument_setter_toggled(self):
        self.instrument_port.setPortPosition()
        
    def button_instrument_up_toggled(self):
        self.instrument_port.positionUp()
        
    def button_instrument_down_toggled(self):
        self.instrument_port.positionDown()
        
    def selectSkinNode(self, target):
        if target:
            self.skinNode = target
            existingSegmentationNode = slicer.util.getNodesByClass('vtkMRMLSegmentationNode')
            for segmentationNode in existingSegmentationNode:
                displayNode_ = segmentationNode.GetDisplayNode()
                displayNode_.SetViewNodeIDs(['vtkMRMLViewNode1'])
                displayNode_.SetVisibility(True)
            displayNode = target.GetDisplayNode()
            displayNode.SetViewNodeIDs(['vtkMRMLViewNode2' , 'vtkMRMLViewNode3'])
            displayNode.SetVisibility(True)
    
    def view2_visble_button_toggled(self, checked):
        if checked:
            self.button1.setText('show')
            if self.skinNode :
                 displayNode = self.skinNode.GetDisplayNode()
                 displayNode.SetVisibility(False)
            
        else:
            self.button1.setText('hide')    
            if self.skinNode :
                 displayNode = self.skinNode.GetDisplayNode()
                 displayNode.SetVisibility(True)
        
    
    
    
