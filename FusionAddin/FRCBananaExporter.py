"""
FRCBananaExporter.py: Main entry point for the FRCBananaSim Fusion 360 Add-In.
Registers the "Export Robot to FRCBananaSim" button in Fusion 360's Utilities / Add-Ins toolbar.
"""

import os
import sys
import traceback

# Add local 'lib' directory to sys.path so modules can import banana_core, fusion, writers
ADDIN_DIR = os.path.dirname(os.path.abspath(__file__))
LIB_DIR = os.path.join(ADDIN_DIR, "lib")
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

try:
    import adsk.core
    import adsk.fusion
    IN_FUSION = True
except ImportError:
    IN_FUSION = False

CMD_ID = "FRCBananaExporter_Cmd"
CMD_NAME = "Export Robot to BananaSim"
CMD_DESCRIPTION = "Exports the active robot assembly to Schema v1 (manifest.json, meshes, and report)."
PANEL_ID = "SolidScriptsAddinsPanel"

handlers = []

_PRIVATE_PACKAGES = ("fusion", "banana_core", "writers")

def _purge_cached_modules():
    """
    Fusion keeps imported modules alive across add-in Stop/Run, so edited .py files would
    otherwise keep running their OLD code. Drop our packages so the next import is fresh.
    """
    for name in list(sys.modules):
        if name in _PRIVATE_PACKAGES or name.startswith(tuple(p + "." for p in _PRIVATE_PACKAGES)):
            del sys.modules[name]

def run(context):
    """Called by Fusion 360 when the add-in is started."""
    if not IN_FUSION:
        print("FRCBananaExporter: Not running inside Autodesk Fusion 360.")
        return


    app = adsk.core.Application.get()
    ui = app.userInterface

    try:
        # Clean up any existing command definition from a previous crash/run
        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()


        # Create new command definition
        cmd_def = ui.commandDefinitions.addButtonDefinition(
            CMD_ID,
            CMD_NAME,
            CMD_DESCRIPTION,
            ""  # icon resource folder (optional)
        )

        # Connect command created event handler
        class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
            def __init__(self):
                super().__init__()


            def notify(self, args):
                try:
                    cmd = args.command
                    on_execute = CommandExecuteHandler()
                    cmd.execute.add(on_execute)
                    handlers.append(on_execute)
                except Exception:
                    ui.messageBox(f"Failed to initialize command:\n{traceback.format_exc()}")

        class CommandExecuteHandler(adsk.core.CommandEventHandler):
            def __init__(self):
                super().__init__()


            def notify(self, args):
                try:
                    _purge_cached_modules()
                    from fusion.export_command import run_export_dialog
                    run_export_dialog(app, ui)
                except Exception:
                    ui.messageBox(f"Export Error:\n{traceback.format_exc()}")


        on_created = CommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        handlers.append(on_created)

        # Add button to the Utilities panel
        panel = ui.allToolbarPanels.itemById(PANEL_ID)
        if panel:
            button = panel.controls.itemById(CMD_ID)
            if not button:
                panel.controls.addCommand(cmd_def)



        app.log("FRCBananaExporter add-in loaded successfully.")

    except Exception:
        if ui:
            ui.messageBox(f"Add-in initialization failed:\n{traceback.format_exc()}")


def stop(context):
    """Called by Fusion 360 when the add-in is stopped."""
    if not IN_FUSION:
        return

    app = adsk.core.Application.get()
    ui = app.userInterface

    try:
        # Remove button from panel
        panel = ui.allToolbarPanels.itemById(PANEL_ID)
        if panel:
            ctrl = panel.controls.itemById(CMD_ID)
            if ctrl:
                ctrl.deleteMe()

        # Remove command definition
        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()

        handlers.clear()
        _purge_cached_modules()
        app.log("FRCBananaExporter add-in unloaded.")


    except Exception:
        if ui:
            ui.messageBox(f"Add-in cleanup failed:\n{traceback.format_exc()}")
