from volume_types import Volume
from collisions_check import cuboid_inside_fuselage, cuboid_collision

def generate_batch(fuselage_range, volume_params, num_cases=10):
    for i in range(num_cases):
        # Sample fuselage dimensions
        fx = random.uniform(*fuselage_range['length'])
        fy = random.uniform(*fuselage_range['width'])
        fz = random.uniform(*fuselage_range['height'])
        # Sample volumes (example: all cuboids)
        volumes = []
        for vp in volume_params:
            v = Volume(
                shape='cuboid',
                length=vp['length'],
                width=vp['width'],
                height=vp['height'],
                x=random.uniform(-fx/2 + vp['length']/2, fx/2 - vp['length']/2),
                y=random.uniform(-fy/2 + vp['width']/2, fy/2 - vp['width']/2),
                z=random.uniform(-fz/2 + vp['height']/2, fz/2 - vp['height']/2),
                constraints=vp.get('constraints', {})
            )
            volumes.append(v)
        # Check collisions
        valid = True
        for idx, v in enumerate(volumes):
            if not cuboid_inside_fuselage(v, (fx, fy, fz)):
                print(f"Volume {idx} not inside fuselage")
                valid = False
        for i in range(len(volumes)):
            for j in range(i+1, len(volumes)):
                if cuboid_collision(volumes[i], volumes[j]):
                    print(f"Volumes {i} and {j} collide")
                    valid = False
        # Export if valid
        if valid:
            print(f"Case {i+1}: Valid configuration. Exporting...")
            # Call your export function here (e.g., to VSP3)
        else:
            print(f"Case {i+1}: Invalid configuration. Skipping.")

# Example usage
fuselage_range = {'length': (10, 15), 'width': (2, 4), 'height': (2, 4)}
volume_params = [
    {'length': 1, 'width': 1, 'height': 1},
    # ... add 9 more volume dicts ...
]
generate_batch(fuselage_range, volume_params, num_cases=10)