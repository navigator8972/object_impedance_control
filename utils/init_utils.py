import numpy as np
import pybullet

#utils function to load pybullet objects. from https://github.com/contactrika/dedo/blob/main/dedo/utils/init_utils.py
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

def load_deform_object_nhk(sim, obj_file_name, texture_file_name,
                       scale, mass, init_pos, init_ori,
                       mu, lbda, damping,
                       friction_coeff, debug):
    """Load object from obj file with pybullet's loadSoftBody()."""
    if debug:
        print('Loading filename', obj_file_name)
    if obj_file_name.endswith('urdf'):
        deform_id = sim.loadURDF(obj_file_name,
            basePosition=init_pos,
            baseOrientation=init_ori,
            #material parameters use ones specified in urdf file
        )
    else:
        # Note: do not set very small mass (e.g. 0.01 causes instabilities).
        deform_id = sim.loadSoftBody(
            mass=mass,  # 1kg is default; bad sim with lower mass
            fileName=obj_file_name,
            scale=scale,
            basePosition=init_pos,
            baseOrientation=init_ori,
            frictionCoeff=friction_coeff,
            # collisionMargin=0.003,  # how far apart do two objects begin interacting
            useSelfCollision=1,
            useFaceContact=True,
            useNeoHookean=1,
            NeoHookeanMu = mu, 
            NeoHookeanLambda = lbda, 
            NeoHookeanDamping = damping, 
            collisionMargin = 0.05,
            repulsionStiffness=2000
        )
    # PyBullet examples for loading and anchoring deformables:
    # https://github.com/bulletphysics/bullet3/examples/pybullet/examples/deformable_anchor.py
    # sim.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
    if texture_file_name is not None:
        texture_id = sim.loadTexture(texture_file_name)
        kwargs = {}
        if hasattr(pybullet, 'VISUAL_SHAPE_DOUBLE_SIDED'):
            kwargs['flags'] = pybullet.VISUAL_SHAPE_DOUBLE_SIDED
        sim.changeVisualShape(
            deform_id, -1, rgbaColor=[1,1,1,1], textureUniqueId=texture_id, **kwargs)
    num_mesh_vertices = get_mesh_data(sim, deform_id)[0]

    if debug:
        print('Loaded deform_id', deform_id, 'with',
              num_mesh_vertices, 'mesh vertices', 'init_pos', init_pos)
    # Pybullet will struggle with very large meshes, so we should keep mesh
    # sizes to a limited number of vertices and faces.
    # Large meshes will load on Linux/Ubuntu, but sim will run too slowly.
    # Meshes with >2^13=8196 vertices will fail to load on OS X due to shared
    # memory limits, as noted here:
    # https://github.com/bulletphysics/bullet3/issues/1965
    assert(num_mesh_vertices < 2**13)  # make sure mesh has less than ~8K verts
    return deform_id

def load_deform_object_mss(sim, obj_file_name, texture_file_name,
                       scale, mass, init_pos, init_ori,
                       bending_stiffness, damping_stiffness, elastic_stiffness,
                       friction_coeff, debug):
    """Load object from obj file with pybullet's loadSoftBody()."""
    if debug:
        print('Loading filename', obj_file_name)

    # Note: do not set very small mass (e.g. 0.01 causes instabilities).
    deform_id = sim.loadSoftBody(
        mass=mass,  # 1kg is default; bad sim with lower mass
        fileName=obj_file_name,
        scale=scale,
        basePosition=init_pos,
        baseOrientation=init_ori,
        springElasticStiffness=elastic_stiffness,
        springDampingStiffness=damping_stiffness,
        springBendingStiffness=bending_stiffness,
        frictionCoeff=friction_coeff,
        # collisionMargin=0.003,  # how far apart do two objects begin interacting
        useSelfCollision=0,
        springDampingAllDirections=1,
        useFaceContact=True,
        useNeoHookean=0,
        useMassSpring=True,
        useBendingSprings=True,
        # repulsionStiffness=10000000,
    )
    
    # PyBullet examples for loading and anchoring deformables:
    # https://github.com/bulletphysics/bullet3/examples/pybullet/examples/deformable_anchor.py
    sim.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
    if texture_file_name is not None:
        texture_id = sim.loadTexture(texture_file_name)
        kwargs = {}
        if hasattr(pybullet, 'VISUAL_SHAPE_DOUBLE_SIDED'):
            kwargs['flags'] = pybullet.VISUAL_SHAPE_DOUBLE_SIDED
        sim.changeVisualShape(
            deform_id, -1, rgbaColor=[1,1,1,1], textureUniqueId=texture_id, **kwargs)
    num_mesh_vertices = get_mesh_data(sim, deform_id)[0]

    if debug:
        print('Loaded deform_id', deform_id, 'with',
              num_mesh_vertices, 'mesh vertices', 'init_pos', init_pos)
    # Pybullet will struggle with very large meshes, so we should keep mesh
    # sizes to a limited number of vertices and faces.
    # Large meshes will load on Linux/Ubuntu, but sim will run too slowly.
    # Meshes with >2^13=8196 vertices will fail to load on OS X due to shared
    # memory limits, as noted here:
    # https://github.com/bulletphysics/bullet3/issues/1965
    assert(num_mesh_vertices < 2**13)  # make sure mesh has less than ~8K verts
    return deform_id

def get_mesh_data(sim, deform_id):
    """Returns num mesh vertices and vertex positions."""
    kwargs = {}
    if hasattr(pybullet, 'MESH_DATA_SIMULATION_MESH'):
        kwargs['flags'] = pybullet.MESH_DATA_SIMULATION_MESH
    num_verts, mesh_vert_positions = sim.getMeshData(deform_id, **kwargs)
    return num_verts, mesh_vert_positions


def print_mesh_data(sim, deform_id, deform_anchored_vertex_ids, step):
    """Prints mesh vertex IDs."""
    num_verts, mesh_vert_positions = get_mesh_data(sim, deform_id)
    print('Step', step, 'anchored mesh locations:')
    num_anchored = len(deform_anchored_vertex_ids)
    for i in range(num_anchored):
        for v in deform_anchored_vertex_ids[i]:
            print(np.array(mesh_vert_positions[v]))