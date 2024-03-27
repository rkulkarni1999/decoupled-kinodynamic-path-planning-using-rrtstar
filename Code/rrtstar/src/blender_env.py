import bpy

boundary_data = None
block_data = []

# Specify the file path
file_path = '/home/ankitmittal/Documents/STUDY/RBE595/HW2a/amittal_p2a/src/sample_maps/map3.txt'  # Replace with the actual file path
def create_block(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b,scale):
    # Scale the dimensions to fit Blender's scene units
    scale_factor = scale # You can adjust this if needed
    # Calculate block center and dimensions
    center_x = (xmax + xmin) / 2
    center_y = (ymax + ymin) / 2
    center_z = (zmax + zmin) / 2
    
    xmin *= scale_factor
    ymin *= scale_factor
    zmin *= scale_factor
    xmax *= scale_factor
    ymax *= scale_factor
    zmax *= scale_factor

    
    dimensions = (xmax - xmin, ymax - ymin, zmax - zmin)

    # Create a new cube object
    bpy.ops.mesh.primitive_cube_add(size=1, enter_editmode=False, align='WORLD', 
        location=(center_x, center_y, center_z))

    # Scale the cube to match the specified dimensions
    bpy.context.object.scale = (dimensions[0], dimensions[1], dimensions[2])

    # Create a new material and assign the RGB color
    mat = bpy.data.materials.new(name="BlockMaterial")
    mat.diffuse_color = (r / 255, g / 255, b / 255, 0.1)
    bpy.context.object.data.materials.append(mat)
    
# Open and read the file
with open(file_path, 'r') as file:
    # Loop through each line in the file
    for line in file:
        # Remove leading and trailing whitespace and split the line into parts
        parts = line.strip().split()

        # Check if the line is empty or a comment (starts with '#')
        if not parts or parts[0].startswith('#'):
            continue  # Skip empty lines and comments

        # Check if it's a boundary line or a block line
        if parts[0] == 'boundary':
            # Parse boundary data
            boundary_data = [float(x) for x in parts[1:]]
            xmin, ymin, zmin, xmax, ymax, zmax= boundary_data
            create_block(xmin, ymin, zmin, xmax, ymax, zmax, 0, 0, 0, scale = 1)
        elif parts[0] == 'block':
            # Parse block data
            block_data.append([float(x) for x in parts[1:]])
            xmin, ymin, zmin, xmax, ymax, zmax, r, g, b = block_data[-1]

for block in block_data:
    xmin, ymin, zmin, xmax, ymax, zmax, r, g, b = block
    create_block(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b, scale = 1)