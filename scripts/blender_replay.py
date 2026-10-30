"""Create a labeled Blender replay from recorded simulation odometry.

Run with Blender's Python interpreter: blender -b -P scripts/blender_replay.py --
  mission.json training_world.world output.blend [--render-video]
The drone is a schematic; its animated pose comes from recorded PX4 odometry.
"""

import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Quaternion, Vector


def material(name, color):
    item = bpy.data.materials.new(name)
    item.diffuse_color = (*color, 1)
    item.use_nodes = True
    principled = item.node_tree.nodes.get('Principled BSDF')
    if principled is not None:
        principled.inputs['Base Color'].default_value = (*color, 1)
        principled.inputs['Roughness'].default_value = 0.72
    return item


def emission_material(name, color, strength=1.0):
    item = bpy.data.materials.new(name)
    item.diffuse_color = (*color, 1)
    item.use_nodes = True
    nodes = item.node_tree.nodes
    nodes.clear()
    output = nodes.new('ShaderNodeOutputMaterial')
    emission = nodes.new('ShaderNodeEmission')
    emission.inputs['Color'].default_value = (*color, 1)
    emission.inputs['Strength'].default_value = strength
    item.node_tree.links.new(emission.outputs['Emission'], output.inputs['Surface'])
    return item


def cube(name, position, dimensions, appearance, parent=None):
    bpy.ops.mesh.primitive_cube_add(size=1, location=position)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    obj.data.materials.append(appearance)
    if parent:
        obj.parent = parent
    return obj


def main():
    args = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    if len(args) < 3:
        raise SystemExit('expected mission.json world.sdf output.blend [--render-video]')
    mission_path, world_path, output = map(Path, args[:3])
    mission = json.loads(mission_path.read_text(encoding='utf-8'))
    points = mission.get('trajectory', [])
    if len(points) < 2:
        raise SystemExit('mission has fewer than two recorded odometry samples')
    # This script is intended for a new background Blender process only.
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_EEVEE'
    scene.render.resolution_x, scene.render.resolution_y = 960, 540
    scene.render.resolution_percentage = 100
    scene.render.fps = 10
    scene.world.color = (0.04, 0.06, 0.09)
    ground = material('Ground', (0.32, 0.40, 0.46))
    obstacle = material('Obstacle', (0.92, 0.17, 0.08))
    border = material('Boundary', (0.10, 0.16, 0.22))
    body = material('Drone', (0.04, 0.08, 0.13))
    rotor = material('Rotors', (0.14, 0.23, 0.30))
    goal_mat = emission_material('Goal', (0.03, 0.95, 0.28), 1.5)
    trail_mat = emission_material('Recorded path', (0.03, 0.30, 1.0), 2.0)
    text_mat = emission_material('Text', (1.0, 1.0, 1.0), 2.0)
    cube('Ground', (4, 0, -0.06), (12, 10, 0.1), ground)
    for model in ET.parse(world_path).findall('./world/model'):
        name = model.get('name', '')
        if name not in ('box1', 'box2', 'cylinder_1', 'wall_n', 'wall_s', 'wall_e', 'wall_w'):
            continue
        pose = [float(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()]
        geometry = model.find('./link/collision/geometry')
        box, cylinder = geometry.find('box'), geometry.find('cylinder')
        if box is not None:
            size = tuple(float(v) for v in box.findtext('size').split())
            # Boundary walls are shown as ground-level borders to keep the path visible.
            if name.startswith('wall_'):
                cube(name + '_border', (pose[0], pose[1], 0.1), (size[0], size[1], 0.2), border)
            else:
                cube(name, pose[:3], size, obstacle)
        elif cylinder is not None:
            bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=float(cylinder.findtext('radius')),
                                                depth=float(cylinder.findtext('length')), location=pose[:3])
            bpy.context.object.name = name
            bpy.context.object.data.materials.append(obstacle)
    gx, gy = mission['goal']
    bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=0.35, depth=0.08, location=(gx, gy, 0.04))
    bpy.context.object.data.materials.append(goal_mat)
    bpy.ops.object.empty_add(type='PLAIN_AXES')
    drone = bpy.context.object
    drone.name = 'Odometry-driven schematic drone'
    cube('Body', (0, 0, 0), (0.35, 0.25, 0.12), body, drone)
    cube('Arm X', (0, 0, 0), (0.9, 0.06, 0.04), body, drone)
    cube('Arm Y', (0, 0, 0), (0.06, 0.9, 0.04), body, drone)
    for x, y in ((0.45, 0), (-0.45, 0), (0, 0.45), (0, -0.45)):
        bpy.ops.mesh.primitive_cylinder_add(vertices=24, radius=0.17, depth=0.015, location=(x, y, 0.06))
        part = bpy.context.object
        part.data.materials.append(rotor)
        part.parent = drone
    curve = bpy.data.curves.new('Recorded path', 'CURVE')
    curve.dimensions = '3D'
    curve.bevel_depth = 0.075
    spline = curve.splines.new('POLY')
    spline.points.add(len(points) - 1)
    for index, item in enumerate(points):
        spline.points[index].co = (*item['position'], 1)
    path = bpy.data.objects.new('Recorded odometry path', curve)
    scene.collection.objects.link(path)
    path.data.materials.append(trail_mat)
    drone.rotation_mode = 'QUATERNION'
    start = points[0]['stamp_ns']
    last_frame = 1
    for item in points:
        frame = 1 + round((item['stamp_ns'] - start) / 1e9 * scene.render.fps)
        if frame < last_frame:
            raise ValueError('trajectory timestamps are not monotonic')
        drone.location = item['position']
        x, y, z, w = item['orientation_xyzw']
        q = Quaternion((w, x, y, z))
        if not math.isfinite(q.magnitude) or q.magnitude < 1e-8:
            raise ValueError('invalid recorded orientation')
        drone.rotation_quaternion = q.normalized()
        drone.keyframe_insert(data_path='location', frame=frame)
        drone.keyframe_insert(data_path='rotation_quaternion', frame=frame)
        last_frame = frame
    scene.frame_end = max(1, last_frame)
    bpy.ops.object.camera_add(location=(-14, -18, 20))
    camera = bpy.context.object
    camera.rotation_euler = (Vector((4, 0, 3)) - camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera.data.type = 'ORTHO'
    camera.data.ortho_scale = 23
    scene.camera = camera
    bpy.ops.object.light_add(type='AREA', location=(0, -5, 18))
    bpy.context.object.data.energy = 3500
    bpy.context.object.data.shape = 'DISK'
    bpy.context.object.data.size = 15
    # Camera-facing labels make the reconstruction and safety result explicit.
    def label(text, x, y, size):
        data = bpy.data.curves.new('Caption', 'FONT')
        data.body = text
        data.size = size
        obj = bpy.data.objects.new('Caption', data)
        scene.collection.objects.link(obj)
        obj.parent = camera
        obj.location = (x, y, -10)
        data.materials.append(text_mat)
    label('SIMULATION TELEMETRY REPLAY  |  schematic drone', -10.8, 5.5, 0.36)
    label('Boundary walls shown as ground borders', -10.8, 5.0, 0.27)
    clearance = mission.get('minimum_navigation_clearance')
    detail = 'unavailable' if clearance is None else f'{clearance:.2f} m'
    label(f"Outcome: {mission.get('reason', 'unknown')}  |  reported min clearance: {detail}",
          -10.8, -5.7, 0.30)
    scene.frame_set(max(1, scene.frame_end // 2))
    output.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(output.resolve()))
    if '--render-video' in args:
        scene.render.image_settings.media_type = 'VIDEO'
        scene.render.image_settings.file_format = 'FFMPEG'
        scene.render.ffmpeg.format = 'MPEG4'
        scene.render.ffmpeg.codec = 'H264'
        scene.render.filepath = str(output.with_suffix('.mp4').resolve())
        bpy.ops.render.render(animation=True)
    else:
        scene.render.image_settings.file_format = 'PNG'
        scene.render.filepath = str(output.with_suffix('.png').resolve())
        bpy.ops.render.render(write_still=True)


if __name__ == '__main__':
    main()
