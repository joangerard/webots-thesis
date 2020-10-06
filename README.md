# Robot Positioning Estimation using ML techniques

This project was developed as a master thesis with the [Machine Learning Group at ULB](mlg.ulb.ac.be). 
It deals with the local robot positioning estimation using:
* Webots robot simulator: Webots2020a
* Python 3.7
* Machine Learning Libraries for Python such that Tensorflow and Keras.
* Neural Networks
* A little bit of Javascript, HTML and bootstrap
* Particles Filter

![Application](https://github.com/joangerard/webots-thesis/blob/master/img/particles.gif "Application")

[Youtube demo here.](https://www.youtube.com/watch?v=fISDutZca5g)
[Youtube demo + presentation here.](https://youtu.be/mf-OTZD2aak)

The following folders can be found in this project: 

* **documents**: it has the presentations made during the development process together with the project documentation under the `documennts/master-thesis-joan-gerard-2020.pdf` file.
* **experiments**: the data and scripts associated with the experiments run.
* **webots-project**: Webots project containing the code used.

This README is oriented to guide the user or the developer through the installation process. 

## User Installation Guidelines

You are a user who wants to have the application running and you are not interested in the code so this section is just for you.

1. Download Webots2020a from [here](https://github.com/cyberbotics/webots/releases/tag/R2020a) and install it.
2. Download Python 3.7 from [here](https://www.python.org/downloads/release/python-377/) and install it if you do not have done it already. **DO NOT** USE `brew` command to install Python because you may have several premission problems while running it from Webots.
3. Clone this repository into your local machine typing `git clone https://github.com/joangerard/webots-thesis` in a terminal.
4.  Install the Python library dependencies following these instructions:

    4.1. It is highly recomended to install a virtual environment in python to isolate the libraries installation.

    4.2.  In order to install a virtual environment please open a terminal and go to the recently cloned repository directory. Then go to the path `webots-project/controllers/pos-prediction` and install a virtual environment with this command.

          // For MacOs and Linux:
          python3 -m pip install --user virtualenv

          // For Windows:
          py -m pip install --user virtualenv

    4.3.  Create a new environment called `env` (or whatever name fits for you) using the command:

          //For MacOs and Linux:
          python3 -m venv env

          //For Windows:
          py -m venv env
          

    4.4.  Activate the environment: venv will create a virtual Python installation in the `env` folder.

          //For MacOs and Linux:
          source env/bin/activate

          //For Windows:
          .\env\Scripts\activate

    4.5.  Confirm that your environment was correctly installed and it is active with the command:

          // For MacOS and Linux:
          which python
          .../env/bin/python <-- you should see this in your terminal

          // For Windows:
          where python
          .../env/bin/python.exe <-- you should see this in your terminal

    4.6.  Now that you have your venvironment up and it is activated install all the project dependencies on it.  

      Go to the path `webots-project/controllers/pos-prediction` and run the command `pip install -r requirements.txt`.  

      This command will install all the project dependencies which are in the `requirements.txt` file.

5.  Go to the folder `webots-project/worlds` you will see several worlds where the experiments were made. The most interesting ones are:
    * `pos-prediction-dev-complex-user.wbt`: This world has configured to run the robot controller inside the Webots tool.
    * `pos-prediction-dev-complex-dev.wbt`: This world has configured to run the robot controller outside the Webots tool using an IDE or Python Command lines, etc. 

    In this section the file called `pos-prediction-user.wbt` will be used. Open it with a double click should lunch the Webots application and the world will be load into it. 

6.  In Webots open the preferences window under the `Webots/Preferences...` option and configure the Python command to be the python executable which is in the recently created virtual environment in the path: `webots-project/controllers/pos-prediction/env/bin/python3`

    ![Preferences](https://github.com/joangerard/webots-thesis/blob/master/img/preferences.png "Preferences Configuration Webots")

7.  Open the Robot Window: click on the robot then right click on it and select the option Show Window Robot. The robot window will be displayed in the left side of the screen.

    ![Show Robot Window](https://github.com/joangerard/webots-thesis/blob/master/img/robot-window.png "Show Robot Window")

## Developer Installation Guidelines

This section is dedicated to the people who want to extend this plugin, reuse it or simply want to see how the code works.

IntelliJ Ultimate 2018.2 is used as IDE tool but any other IDE should be similarly configurable.

The robot controller code was programmed using Python 3.7 and it is under the directory `webots-project/controllers/pos-prediction`. 

The robot window plugin was programmed using JavaScript 6 and it is under the directory `/webots-project/plugins/robot_windows/pos-prediction`. 

### Installation

1. Follow steps 1 to 4 from the User Guidelines section.
2.  Configure the following environment variables in MacOs using the `export` command

    | Env Variable        | Typical Value                                 |
    | ------------------- |:---------------------------------------------:|
    | WEBOTS_HOME         | /Applications/Webots.app                      |
    | DYLD_LIBRARY_PATH   | add ${WEBOTS_HOME}/lib/controller             |
    | PYTHONPATH          | add ${WEBOTS_HOME}/lib/controller/python3X    |

    For instance you could add these lines to the `~/.bash_profile`:

    ```
    export WEBOTS_HOME="/Applications/Webots.app"
    export DYLD_LIBRARY_PATH="${WEBOTS_HOME}/lib/controller:$DYLD_LIBRARY_PATH"
    export PYTHONPATH="${WEBOTS_HOME}/lib/controller/python37:$PYTHONPATH"
    export PYTHONIOENCODING="UTF-8"
    ```

    And execute the `source ~/.bash_profile` command and that is it!

    For other operative systems please refer to [this](https://cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=macos&tab-language=python) link. 

3.  Configure IntelliJ to lunch the robot controller.

    3.1. Open the `webots-project/controllers/pos-prediction` project on IntelliJ.

    3.2. Click on `File/Project Structure...`, click on the `Add Content Root`, and select the Python version of Webots. In MacOs this can be found here: `Applications/Webots.app/lib/controller/python37`.
  ![Add Content Root](https://github.com/joangerard/webots-thesis/blob/master/img/add-content-root.png "Add Content Root")

    3.3.  Click on `Run/Run.../Edit Configurations...`, in the environment variables add:

          ```
          PYTHONUNBUFFERED=1;
          DYLD_LIBRARY_PATH=/Applications/Webots.app/lib/controller
          ```  
      ![Running Configurations](https://github.com/joangerard/webots-thesis/blob/master/img/run-main.png "Running Configurations")    

      *N.B.* Make sure you selected the interpreter that is associated with your virtual environment. In the picture below, the virtual environment is called `pos-prediction` and it was created with the IDE selecting the option `File/Project Structure.../SDKs` and click on `+` sign to add another one. This is equivalent to the venv command previously used. It has the same purpose of isolating the installed libraries.


4. Open the `webots-project/worlds/pos-prediction-dev-complex-dev.wbt` using Webots(a right click is usually enough) and click on the play button. This will do nothing but wait an external controller to run the simulation.

5. Run the `pos-prediction.py` file on IntelliJ. You can simply press the `Run` button or right click on the file name and press `Run`.

6. You will see the simulation starts to run on Webots if everything was success.

### Comand Line Arguments

It is possible to specify some arguments to pass to the controller before it runs like the initial robot possition or the amount of particles, etc. In order to do so IntelliJ will be used to pass arguments each time we run the controller. This can be done through the Parameters field in the Running Configuration Window (Last image of step 2 in this section).

The list of arguments are listed below:

```
usage: pos-prediction.py [-h] [--initx INITX] [--inity INITY]
                         [--experiment_duration_steps EXPERIMENT_DURATION_STEPS]
                         [-pn PARTICLES_NUMBER] [--sigma_xy SIGMA_XY]
                         [--sigma_theta SIGMA_THETA] [--calculate_pred_error]
                         [--calculate_odo_error]
                         [--pred_error_file PRED_ERROR_FILE]
                         [--pred_odo_file PRED_ODO_FILE] [--go_straight]
                         [--capture_data] [--global_localization]

optional arguments:
  -h, --help            show this help message and exit
  --initx INITX         Initial X position of robot
  --inity INITY         Initial Y position of robot
  --experiment_duration_steps EXPERIMENT_DURATION_STEPS
                        Max number of duration steps
  -pn PARTICLES_NUMBER, --particles_number PARTICLES_NUMBER
                        Number of particles
  --sigma_xy SIGMA_XY   Sigma XY that controls the SDE of the x,y coordinates
  --sigma_theta SIGMA_THETA
                        Sigma Theta that controls the SDE of the robot angle
  --calculate_pred_error
                        Store the prediction error in form of .pkl file
  --calculate_odo_error
                        Store the odometry error in form of .pkl file
  --pred_error_file PRED_ERROR_FILE
                        Name of the file where to save the prediction error
  --pred_odo_file PRED_ODO_FILE
                        Name of the file where to save the odometry error
  --go_straight         Let the robot go straight
  --capture_data        Capture data mode on
  --global_localization
                        Global localization problem

```

#### Examples of Arguments

This command tells the robot to start in position (0.5, 0.5) on the arena.

```
pos-prediction.py --initx 0.5 --inity 0.5
```

This command tells the robot to use 1000 particles with an standar deviation of 0.01 metters in the coordinates and 20 degrees in the angle.

```
pos-prediction.py --particles_number 1000 --sigma_xy 0.01 --sigma_theta 20
```

This command captures the prediction error at each time step and it saves it into a .pkl file. Then the array of values can be loaded from the file into a variable using the Pickle Python Library. Each position of the array contains the value of sqrt((x_pred - x_true)^2 + (y_pred - y_true)^2)

```
pos-prediction.py --calculate_pred_error --pred_error_file "data_30_partices_001_Sigma.pkl"
```

This was used for obtaining comparable results of the experiments.

