# Setting up a VSCode Development Environment
This guide will walk you through how to set up your computer to make code changes and push them to the RoboRio. You should make sure all of these instructions work before you spend a lot of time changing any code to know that your end-to-end testing loop with a robot will work.

## Get Set Up With Git
TODO

1. Git `Team100/all24` repo checked out.

## Get VSCode + WPILib Set Up
TODO

1. You might need to install [Java](https://marketplace.visualstudio.com/items?itemName=vscjava.vscode-java-pack) manually to get autocomplete etc?


## Getting VSCode Workspace Working
In VSCode, go to `File > Open Folder` and select the `swerve100` folder. 

When you open the swerve100 project, VS Code should show a message in the bottom-right corner suggesting that you open the "workspace" instead, which includes lib:

<img src="readme_img/openworkspace.png" width=350/>

When you have VS Code correctly set up, the Explorer view should contain two top-level folders: "swerve100" and "lib":

<img src="readme_img/workspace.png" width=350/>
TODO.

## Building the RoboRIO Code
Use the WPI extensions to run the `gradlew` build:
1. Press `CTRL + SHIFT + P` to get the VSCode Command Mode.
1. Search for `Build Robot Code`
1. Select `swerve100` when it asks.
1. A terminal should pop up that runs the build. If successful, you will see this:
    ```
    BUILD SUCCESSFUL in 2s``
    6 actionable tasks: 1 executed, 5 up-to-date
    Watched directory hierarchies: [C:\Users\Engineering Student\src\all24\comp\swerve100]
    ```
TODO: what if the build is not successful?

## Simulating the RoboRIO
Use the WPI extensions to kick off the Simulator. Good overall instructions are available [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html). 
1. Press `CTRL + SHIFT + P` to get the VSCode Command Mode.
1. Search for `Simulate Robot Code`
1. Select `swerve100` when it asks.
1. It will ask you what simulations to run. Make sure `Sim GUI` is selected, and nothing else.
1. You should get the robot simulation GUI, which looks like this:
![Simulation GUI](readme_img/sim_gui.png)
1. Make sure you can debug your code in simulation.
    1. If you aren't familiar with debugging in VSCode, read this: TODO.
    1. Set a debug breakpoint on:
        ```java
        public void robotPeriodic() 
        ```
    1. The simulation should trip your debugger since it calls this function every few ms. Then you are ready to go!

### Using the Simulator
1. Make sure you have the Field view available. This will allow you to "drive" the robot virtually. If you don't, go to `Network Tables > Field` ![Field View](readme_img/field_view.png).
    1. If you don't see the Field view widget/option, make sure your Robot code is running flawlessly (and no breakpoints were hit).
    1. Learn more about using the Field widget [here](https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html).
1. Try to drive the Robot. You will need to setup your Keyboard as a Joystick if you haven't yet. 
    1. Click and drag Keyboard 0 to Joystick[0] like this: 
    ![Drag Keyboard](readme_img/drag_keyboard.png).
    1. Select `Teleoperated` from the `Robot State` widget.
    1. You should be able to use the `A-W-S-D` keys to move in 2 dimensions (Axis 0 and Axis 1).
    1. You should be able to use the `E-R` keys to rotate the robot (Axis 2).
1. Try to put the Robot in autonomous mode. This will cause it to execute known trajectories.
    1. First, let's get debugging working by increasing the Log Level. 
        1. Go to `Network Tables > SmartDashboard > Log Level`.
        ![Alt text](readme_img/log_level.png)
        1. Select `TRACE` level, which should be the maximum
    1. Click on `Autonomous` in the `Robot State` widget.
    1. The robot should start driving around, with these ugly trajectories drawn:
    ![Alt text](readme_img/ugly_traj.png)
    1. Click the Hamburger Menu in the top-right of the field window, select Trajectory, and turn off the Arrows. You should get a nicer one like this:
    ![Alt text](readme_img/nice_traj.png)

## Deploying to the RoboRIO
Once you have your code compiling locally there is one more step before you start changing things: making sure you can deploy to a RoboRIO. 

1. Ask a mentor for a RoboRIO you can use and get their help to power it up. The green light should come on.
1. Connect the RoboRIO to your laptop with a USB cable, like these photos:
    1. ![](readme_img/usb_1.jpg)
    1. ![](readme_img/usb_2.jpg)
1. Click the "W" button (or go to the search) in VSCode and find `WPILIB: Deploy Robot Code`
    1. ![deploy](readme_img/wpilib_deploy.png)
1. Select `swerve100` and then watch the build happen in the terminal. You should get `BUILD SUCCESSFUL` and the RioLog should pop up on the right after the build. 
    1. ![rio_log](readme_img/rio_log.png)

You are (hopefully!) getting a bunch of errors because we just deployed "production" code to a test RoboRIO, so it can't find motors, devices, etc. That's good! Move forward to the next step, [controlling your first motor](README_2_MOTOR.md)