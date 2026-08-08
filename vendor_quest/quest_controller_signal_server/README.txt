Quest 3 Controller Signal Server
================================

This version does NOT hard-code VR_Teleop.

Startup workflow
----------------
1. Double-click:
       start_quest_controller_signal_server.bat

2. The launcher displays:
       conda env list

3. Enter the Conda environment name yourself, for example:
       VR_Teleop

4. After activation, the main menu appears:

       1. Terminal Mode
       2. GUI Mode
       3. Connection Test
       4. Install Dependencies
       5. Exit

Option 4 installs dependencies into the environment selected at startup.

Recommended first use
---------------------
1. Connect Quest 3 through USB.
2. Double-click the BAT launcher.
3. Enter the desired Conda environment name.
4. Select 4 to install dependencies into that environment.
5. Select 3 to check Python, ADB and port forwarding.
6. Select 2 to launch the GUI.

Daily use
---------
1. Connect Quest 3 through USB.
2. Double-click the BAT launcher.
3. Enter the desired Conda environment name.
4. Select 2.

WebSocket
---------
Server:
    ws://127.0.0.1:8766

USB forwarding:
    adb reverse tcp:8766 tcp:8766
