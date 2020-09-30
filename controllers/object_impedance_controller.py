"""
An implementation of MultiAgent Object-level Impedance Controller
"""

import numpy as np

import object_impedance_control.controllers.utils as utils

class MultiAgentObjectImpedanceController():

    def __init__(self,  n_agents=3,                                     #number of agents
                        object_stiffness_trans=80,                      #target object translational stiffness                    
                        object_stiffness_rot=10,                        #target object rotational stiffness
                        desired_object_pos=np.zeros(3),                 #desired object position
                        desired_object_rot=np.array([0, 0, 0, 1]),      #desired object orientation quaternion (scalar last format)
                        agent_link_stiffness=20):                        #stiffness of the links between agents and centroid
        
        self._n_agents = n_agents
        self._object_stiffness_trans = object_stiffness_trans * np.eye(3)
        self._object_stiffness_rot = object_stiffness_rot * np.eye(3)
        self._desired_object_pos = desired_object_pos
        self._desired_object_rot = desired_object_rot
        self._agent_link_stiffness = [agent_link_stiffness * np.diag([1.0, 0.25, 0.25])] * self._n_agents
        return
    
    def step(self, agent_pos, agent_force=None):
        """
        Determine virtual frame from given agent position and compute wrench for each agent. agent_force can be used as
        weight to determine the geometry centroid

        Input
        agent_pos:      n x 3 array - n is number of agent  n>=3 to decide a full reference frame
        agent_force:    n size array - norm of force sensed at each agent. Use an even distribution if None

        Output
        agent_wrench:   n x 3 array - wrench to execute at each agent, only force part is concerned since agent is modeled as a point
                                        the offset from contact point on a rigid link should be handled by the jacobian of manipulator itself
        """

        assert(np.array(agent_pos).shape[0] == self._n_agents)
        agent_pos = np.array(agent_pos)

        if agent_force is not None:
            assert(len(agent_force) == self._n_agents)
            if np.linalg.norm(agent_force) > 1e-4:
                weights = abs(agent_force)/sum(abs(agent_force))
            else:
                weights = np.array([1./self._n_agents]*self._n_agents)
        else:
            weights = np.array([1./self._n_agents]*self._n_agents)

        virtual_frame_origin = np.sum(weights[:, np.newaxis] * agent_pos, axis=0)
        # print('agent_pos:', agent_pos)
        # print('virtual_origin:', virtual_frame_origin)

        #construct rotation matrix, use only first three agents...
        rx = (agent_pos[2] - agent_pos[0]) / np.linalg.norm(agent_pos[2] - agent_pos[0])
        x21rx = np.cross(agent_pos[1] - agent_pos[0], rx)
        rz = x21rx / np.linalg.norm(x21rx)
        virtual_frame_rot = np.array([rx, np.cross(rz, rx), rz]).T
        # print('virtual_rot:', virtual_frame_rot)
        
        #get grasp force for each agent
        grasp_forces = np.zeros((self._n_agents, 3))
        for i in range(self._n_agents):
            #build a local reference frame according to current link vector
            delta_x = virtual_frame_origin - agent_pos[i]
            # print('delta_x:', delta_x)
            tmp_x_axis = delta_x / np.linalg.norm(delta_x)
            tmp_y_axis = np.cross(tmp_x_axis, virtual_frame_rot[:, 1])
            tmp_y_axis = tmp_y_axis / np.linalg.norm(tmp_y_axis)
            tmp_z_axis = np.cross(tmp_x_axis, tmp_y_axis)
            tmp_z_axis = tmp_z_axis / np.linalg.norm(tmp_z_axis)           #should be avoidable but put here just to ensure...
            
            tip_to_global = np.array([tmp_x_axis, tmp_y_axis, tmp_z_axis]).T

            #specified link stiffness is local so need to transform it to the global frame
            global_agent_stiffness = tip_to_global.dot(self._agent_link_stiffness[i]).dot(tip_to_global.T)

            grasp_forces[i, :] = global_agent_stiffness.dot(delta_x)
        
        #get wrench for object elastic behaviour
        #translational part
        trans_err = self._desired_object_pos - virtual_frame_origin

        trans_forces = np.array([weights[i] * self._object_stiffness_trans.dot(trans_err) for i in range(self._n_agents)])

        desired_rot_mat = utils.quaternion_to_matrix(self._desired_object_rot)
        #using cross product to characterize orientation error
        r_rd_prod = np.cross(virtual_frame_rot, desired_rot_mat, axisa=1, axisb=1)
        omega_vec_origin = np.sum(r_rd_prod, axis=1)

        omega = omega_vec_origin.dot(self._object_stiffness_rot)


        #decompose attitude
        #An = rz * rz' / ||rz||^2
	    #At = I - An
        rz = virtual_frame_rot[:, 2]
        rz_square = np.linalg.norm(rz)**2
        # An = np.zeros(3)
        # At = np.zeros(3)
        # for i in range(3):
        #     for j in range(3):
        #         An[i, j] = rz[i] * rz[j] / rz_square
        #         if i == j:
        #             At[i, j] = 1 - An[i, j]
        #         else:
        #             At[i, j] = - An[i, j]
        An = np.array([[rz[i]*rz[j] / rz_square for j in range(3)] for i in range(3)])
        At = np.eye(3) - An

        At_omega = omega.dot(At)
        An_omega = omega.dot(An)

        rz_cross_at_omega = np.cross(rz, At_omega)
        rz_cross_at_omega_norm = np.linalg.norm(rz_cross_at_omega)

        rot_forces = np.zeros((self._n_agents, 3))
        for i in range(self._n_agents):
            l_ni = virtual_frame_origin - agent_pos[i]
            #l_ti = rz_cross_At_omega * l_ni.Dot(rz_cross_At_omega) / (rz_cross_At_omega.Norm() * rz_cross_At_omega.Norm())
            l_ti = np.zeros(3) if rz_cross_at_omega_norm < 1e-2 else rz_cross_at_omega * (l_ni.dot(rz_cross_at_omega) / rz_cross_at_omega_norm**2)

            #I guess we need weights here but the legacy code commented out omega[i]...
            rot_forces[i, :] = (np.cross(l_ni, An_omega) + np.cross(l_ti, An_omega)) * weights[i]

        return grasp_forces + trans_forces + rot_forces

    def set_object_stiffness_trans(self, object_stiffness_trans):
        self._object_stiffness_trans = object_stiffness_trans
        return
    
    def set_object_stiffness_rot(self, object_stiffness_rot):
        self._object_stiffness_rot = object_stiffness_rot
        return
    
    def set_object_stiffness(self, trans, rot):
        self.set_object_stiffness_trans(trans)
        self.set_object_stiffness_rot(rot)
        return

    def set_desired_object_pos(self, pos):
        self._desired_object_pos = pos
        return
    
    def set_desired_object_rot(self, rot):
        self._desired_object_rot = rot
        return
    
    def set_agent_link_stiffness(self, i, stiffness):
        #must specify a stiffness matrix
        assert(stiffness.shape == (3, 3))
        assert(i < self._n_agents)
        self._agent_link_stiffness[i] = stiffness
        return