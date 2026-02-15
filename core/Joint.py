# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019
Modified on Sun Jan 17 2021

@author: syuntoku
@author: spacemaster85
"""

import adsk, re, traceback
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils

class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'

    def make_joint_xml(self):
        """
        Generate the joint_xml and hold it by self.joint_xml
        """
        joint = Element('joint')
        joint.attrib = {'name':self.name, 'type':self.type}

        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link':self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link':self.child}
        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': '100', 'velocity': '100'}

        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        """
        Generate the tran_xml and hold it by self.tran_xml


        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface
        """

        tran = Element('transmission')
        tran.attrib = {'name':self.name + '_tran'}

        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'

        joint = SubElement(tran, 'joint')
        joint.attrib = {'name':self.name}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'

        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name':self.name + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'

        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])


def make_joints_dict(root, msg):
    """
    joints_dict holds parent, axis and xyz informatino of the joints


    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status

    Returns
    ----------
    joints_dict:
        {name: {type, axis, upper_limit, lower_limit, parent, child, xyz}}
    msg: str
        Tell the status
    """

    joint_type_list = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical',
    'PinSlot', 'Planner', 'Ball']  # these are the names in urdf

    joints_dict = {}

    # Collect joints from the most complete collection available.
    # root.joints may miss nested/context joints depending on assembly structure.
    raw_joints = []
    all_joints = getattr(root, 'allJoints', None)
    if all_joints:
        raw_joints = [j for j in all_joints]
    else:
        raw_joints = [j for j in root.joints]

    warning_msgs = []

    for joint in raw_joints:
        if joint.isLightBulbOn :
            joint_dict = {}
            joint_type = joint_type_list[joint.jointMotion.jointType]
            joint_dict['type'] = joint_type
    
            # switch by the type of the joint
            joint_dict['axis'] = [0, 0, 0]
            joint_dict['upper_limit'] = 0.0
            joint_dict['lower_limit'] = 0.0
    
            # support  "Revolute", "Rigid" and "Slider"
            if joint_type == 'revolute':
                joint_dict['axis'] = [round(i, 6) for i in \
                    joint.jointMotion.rotationAxisVector.asArray()] ## In Fusion, exported axis is normalized.
                max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
                min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled
                if max_enabled and min_enabled:
                    joint_dict['upper_limit'] = round(joint.jointMotion.rotationLimits.maximumValue, 6)
                    joint_dict['lower_limit'] = round(joint.jointMotion.rotationLimits.minimumValue, 6)
                elif max_enabled and not min_enabled:
                    warning_msgs.append(joint.name + ' is not set its lower limit. Skipping this joint.')
                    continue
                elif not max_enabled and min_enabled:
                    warning_msgs.append(joint.name + ' is not set its upper limit. Skipping this joint.')
                    continue
                else:  # if there is no angle limit
                    joint_dict['type'] = 'continuous'
    
            elif joint_type == 'prismatic':
                joint_dict['axis'] = [round(i, 6) for i in \
                    joint.jointMotion.slideDirectionVector.asArray()]  # Also normalized
                max_enabled = joint.jointMotion.slideLimits.isMaximumValueEnabled
                min_enabled = joint.jointMotion.slideLimits.isMinimumValueEnabled
                if max_enabled and min_enabled:
                    joint_dict['upper_limit'] = round(joint.jointMotion.slideLimits.maximumValue/100, 6)
                    joint_dict['lower_limit'] = round(joint.jointMotion.slideLimits.minimumValue/100, 6)
                elif max_enabled and not min_enabled:
                    warning_msgs.append(joint.name + ' is not set its lower limit. Skipping this joint.')
                    continue
                elif not max_enabled and min_enabled:
                    warning_msgs.append(joint.name + ' is not set its upper limit. Skipping this joint.')
                    continue
            elif joint_type == 'fixed':
                pass
            
    
            def get_parent(occ): 
            # function to find the root component of the joint. This is necessary for the correct component name in the urdf file
                if occ.assemblyContext != None:
                    #print(occ.name)
                    occ = get_parent(occ.assemblyContext)
                return occ
    
            if joint.occurrenceTwo != None and joint.occurrenceOne != None and joint.occurrenceOne.isLightBulbOn:
                occ_two = get_parent(joint.occurrenceTwo)
                occ_one = get_parent(joint.occurrenceOne)

                # Prefer base_link as parent when present to keep the URDF tree direction stable.
                occ_two_name = occ_two.name
                occ_one_name = occ_one.name
                if "base_link" in occ_one_name and "base_link" not in occ_two_name:
                    parent_occ = occ_one
                    child_occ = occ_two
                else:
                    parent_occ = occ_two
                    child_occ = occ_one

                if "base_link" in parent_occ.name:
                    joint_dict['parent'] = 'base_link'
                else:
                    joint_dict['parent'] = re.sub('[ :()]', '_', parent_occ.name)

                if "base_link" in child_occ.name:
                    joint_dict['child'] = 'base_link'
                else:
                    joint_dict['child'] = re.sub('[ :()]', '_', child_occ.name)
            else:
                # Fusion often creates helper rigid/fixed joints tied to grounded/root context.
                # They are not needed for URDF kinematic chains, so skip them silently.
                if joint_dict['type'] != 'fixed':
                    warning_msgs.append(joint.name + ' has invalid occurrences and was skipped.')
                continue
            
            def getJointOriginWorldCoordinates(joint :adsk.fusion.Joint):
            # Function to transform the joint origin coordinates which are in the component context into world coordinates
            # Thanky you for the help in the fusion forum
            # https://forums.autodesk.com/t5/fusion-360-api-and-scripts/how-to-get-the-joint-origin-in-world-context/m-p/10011971/highlight/false#M12401
                def getMatrixFromRoot(root_occ) -> adsk.core.Matrix3D:
                    mat = adsk.core.Matrix3D.create()
    
                    occ = adsk.fusion.Occurrence.cast(root_occ)
                    if not occ:
                        return mat # root

                    def getParentOccs(joint):
                        occs_list = []
            
                        if joint != None:    
                            occs_list.append(joint)
             
                        if joint.assemblyContext != None:
                            occs_list = occs_list + getParentOccs(joint.assemblyContext)
            
                        return occs_list
                    
                    occs = getParentOccs(root_occ)
                    mat3ds = [occ.transform for occ in occs if occ!= None]
                    #mat3ds.reverse()
                    for mat3d in mat3ds:
                        mat.transformBy(mat3d)
                    return mat
    
                # mat :adsk.core.Matrix3D = getMatrixFromRoot(joint.occurrenceOne)
                # ori1 :adsk.core.Point3D = joint.geometryOrOriginOne.origin.copy()
                # ori1.transformBy(mat)
    
                mat = getMatrixFromRoot(joint.occurrenceTwo)
                ori2 :adsk.core.Point3D = joint.geometryOrOriginTwo.origin.copy()
                ori2.transformBy(mat)
                return  ori2 #ori1,
    
            try:
                xyz_of_joint = getJointOriginWorldCoordinates(joint)
                joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_of_joint.asArray()]  # converted to meter
                #print(f"xyz : {joint_dict['xyz']}")
    
            except:
                print('Failed:\n{}'.format(traceback.format_exc()))
                try:
                    if type(joint.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
                        data = joint.geometryOrOriginTwo.geometry.origin.asArray()
                    else:
                        data = joint.geometryOrOriginTwo.origin.asArray()
                    joint_dict['xyz'] = [round(i / 100.0, 6) for i in data]  # converted to meter
                except:
                    warning_msgs.append(joint.name + " doesn't have joint origin and was skipped.")
                    continue

            joints_dict[joint.name] = joint_dict
    if len(joints_dict) == 0 and len(raw_joints) > 0:
        msg = 'No valid joints found. Check that joints are visible and connect two occurrences.'
    elif warning_msgs:
        msg = 'Successfully create URDF file\nWarnings:\n' + '\n'.join(warning_msgs)
    return joints_dict, msg
