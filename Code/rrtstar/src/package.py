
import bpy
import sys
import site

# PATH CONFIGURATION
user_site_packages =site.getusersitepackages()
sys.path.append(user_site_packages) #For pip installed dependencies
sys.path.append('./src')
sys.path.append('/home/ankitmittal/Documents/STUDY/RBE595/aerial/lib/python3.10/site-packages')
#sys.path.append('/home/oliver//Simulator/blender-sim/src') #For custom python files import using absolute path
#sys.path.append('/home/oliver/blender-3.4.1-linux-x64/3.4/project_dependencies/blender_env/lib/python3.10/site-packages')