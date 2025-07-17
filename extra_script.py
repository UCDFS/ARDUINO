import os
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

def cleanup_nextion_files(*args, **kwargs):
    print("Searching for and removing Nextion upload files...")
    libdeps_dir = os.path.join(env.get("PROJECT_DIR"), ".pio", "libdeps")
    
    if os.path.exists(libdeps_dir):
        for root, dirs, files in os.walk(libdeps_dir):
            if "Nextion" in root:
                for file_to_remove in ["NexUpload.h", "NexUpload.cpp"]:
                    file_path = os.path.join(root, file_to_remove)
                    if os.path.exists(file_path):
                        os.remove(file_path)
                        print(f"Removed: {file_path}")

env.AddPreAction("buildprog", cleanup_nextion_files)
