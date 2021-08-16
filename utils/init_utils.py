import pybullet

#utils function to load a rigid object. from contactrika/dedo
def load_rigid_object(sim, obj_file_name, scale, mass, init_pos, init_ori, texture_file=None,
                      rgba_color=None):
    """Load a rigid object from file, create visual and collision shapes."""
    if obj_file_name.endswith('.obj'):  # mesh info
        xyz_scale = [scale, scale, scale]
        viz_shape_id = sim.createVisualShape(
            shapeType=pybullet.GEOM_MESH, rgbaColor=rgba_color,
            fileName=obj_file_name, meshScale=xyz_scale)
        col_shape_id = sim.createCollisionShape(
            shapeType=pybullet.GEOM_MESH,
            fileName=obj_file_name, meshScale=xyz_scale)
        rigid_id = sim.createMultiBody(
            baseMass=mass,  # mass==0 => fixed at the position where it is loaded
            basePosition=init_pos,
            # useMaximalCoordinates=1, # TODO Delete me
            baseCollisionShapeIndex=col_shape_id,
            baseVisualShapeIndex=viz_shape_id,
            baseOrientation=init_ori)
    elif obj_file_name.endswith('.urdf'):  # URDF file
        rigid_id = sim.loadURDF(
            obj_file_name, init_pos, init_ori,
            useFixedBase=1, globalScaling=scale)
    else:
        print('Unknown file extension', obj_file_name)
        assert(False), 'load_rigid_object supports only obj and URDF files'
    n_jt = sim.getNumJoints(rigid_id)
    # print(n_jt)
    if texture_file is not None:
        texture_id = sim.loadTexture(texture_file)
        kwargs = {}
        if hasattr(pybullet, 'VISUAL_SHAPE_DOUBLE_SIDED'):
            kwargs['flags'] = pybullet.VISUAL_SHAPE_DOUBLE_SIDED
        for i in range(-1, n_jt):
            sim.changeVisualShape(
                rigid_id, i, rgbaColor=[1,1,1,1], textureUniqueId=texture_id, **kwargs)
    return rigid_id