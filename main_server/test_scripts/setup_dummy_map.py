import os
import numpy as np
from PIL import Image

def create_dummy_map():
    # 1. Create a 200x200 white map (10m x 10m with 0.05 resolution)
    width, height = 200, 200
    # 255 = Free. No border this time to avoid inflation issues.
    map_data = np.full((height, width), 255, dtype=np.uint8)
    
    # Save as PGM
    base_dir = os.path.dirname(os.path.abspath(__file__))
    pgm_path = os.path.join(base_dir, "test_map.pgm")
    
    img = Image.fromarray(map_data)
    img.save(pgm_path)
    print(f"Created dummy PGM map at {pgm_path}")
    
    # 2. Create YAML config
    yaml_content = f"""image: {pgm_path}
mode: trinary
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    yaml_path = os.path.join(base_dir, "test_map.yaml")
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"Created dummy YAML map at {yaml_path}")
    
    return yaml_path

if __name__ == "__main__":
    create_dummy_map()
