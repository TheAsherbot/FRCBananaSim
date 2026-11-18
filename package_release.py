import os
import shutil
import zipfile

def package():
    repo_root = os.path.dirname(os.path.abspath(__file__))
    build_dir = os.path.join(repo_root, "build", "Release")
    exe_path = os.path.join(build_dir, "FRCBananaSim.exe")
    pdb_path = os.path.join(build_dir, "FRCBananaSim.pdb")
    resources_dir = os.path.join(repo_root, "FRCBananaSim", "Resources")
    
    if not os.path.exists(exe_path):
        print(f"Error: Executable not found at {exe_path}")
        return False

    staging_dir = os.path.join(repo_root, "build", "FRCBananaSim-Release-Windows-x64")
    if os.path.exists(staging_dir):
        shutil.rmtree(staging_dir)
    os.makedirs(staging_dir, exist_ok=True)

    # 1. Copy Executable & PDB
    shutil.copy2(exe_path, os.path.join(staging_dir, "FRCBananaSim.exe"))
    if os.path.exists(pdb_path):
        shutil.copy2(pdb_path, os.path.join(staging_dir, "FRCBananaSim.pdb"))

    # 2. Copy Resources directory (exclude heavy 500MB unused CAD .glb files by default)
    dest_resources = os.path.join(staging_dir, "Resources")
    shutil.copytree(resources_dir, dest_resources, ignore=shutil.ignore_patterns("*.glb", "*.obj"))

    # 3. Create helpful README.txt
    readme_content = """========================================================================
FRC BANANA SIMULATOR — ROBOT COMP 2 ARCADE SHOOTOUT
========================================================================

HOW TO RUN:
1. Double-click "FRCBananaSim.exe" to play!
   - Note: FRCBananaSim.exe is also 100% self-contained. You can copy or move
     FRCBananaSim.exe anywhere (even standalone) and it will run using its
     embedded robot definitions.
   - If the "Resources" folder is next to the executable, it will load the
     customizable files from disk.

GAME CONTROLS:
- W / A / S / D or Arrow Keys : Drive & Strafe swerve chassis
- Q / E                      : Rotate / Yaw robot left and right
- Space / Left Shift         : Deploy Intake & Run intake rollers (Collect Cargo Balls)
- F / RMB / V                : Run Feeder Funnel (Index balls upward toward shooter)
- LMB / C                    : Spin Flywheel & Fire Cargo Balls into the Alliance Goal
- Z                          : Stride / Sprint Boost
- Tab                        : Toggle Free-Fly vs Orbit Camera
- Esc                        : Pause Game
- R                          : Reset Field (available in Free Practice or Menus)

========================================================================
"""
    with open(os.path.join(staging_dir, "README.txt"), "w", encoding="utf-8") as f:
        f.write(readme_content)

    # 4. Create ZIP archive
    zip_path = os.path.join(repo_root, "build", "FRCBananaSim-Release-Windows-x64.zip")
    if os.path.exists(zip_path):
        os.remove(zip_path)

    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zipf:
        for root, dirs, files in os.walk(staging_dir):
            for file in files:
                abs_file = os.path.join(root, file)
                rel_file = os.path.relpath(abs_file, staging_dir)
                zipf.write(abs_file, os.path.join("FRCBananaSim", rel_file))

    zip_size_mb = os.path.getsize(zip_path) / (1024 * 1024)
    print(f"Successfully packaged release bundle: {zip_path} ({zip_size_mb:.2f} MB)")
    return True

if __name__ == "__main__":
    package()
