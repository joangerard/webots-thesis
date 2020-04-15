# Robot Positioning Estimation using ML techniques

This project is about robot positioning estimation using:
* Webots robot simulator: Webots2020a
* Python 3.7
* Machine Learning Libraries for Python such that Tensorflow and Keras.
* Neural Networks
* A little bit of Javascript, HTML and bootstrap
* Particles Filter

The following folders can be found in this project: 

* documents: it has the presentations made during the development process together with the project documentation under the `documennts/thesis-doc/main.pdf` file.
* experiments: the data associated with the experiments run.
* webots-project: Webots project containing the code used.

This README is oriented to guide the user or the developer through the installation process. 

## User Guidelines

You are a user who wants to have the application running and you are not interested in the code so this section is just for you.

1. Download Webots2020a from [here](https://github.com/cyberbotics/webots/releases/tag/R2020a) and install it.
2. Download Python 3.7 from [here](https://www.python.org/downloads/release/python-377/) and install it if you do not have done it already. DO NOT USE `brew` command to install Python because you may have several premission problems while running it from Webots.
3. Clone this repository into your local machine typing `git clone https://github.com/joangerard/webots-thesis` in a terminal.
4. Install the Python library dependencies.
  * It is highly recomended to install a virtual environment in python to isolate the libraries installation.
    
    In order to install a virtual environment please open a terminal and go to the recently cloned repository directory. Then go to the path `webots-project/controllers/pos-prediction` and install virtual environment.

    For MacOs and Linux:
    ```python3 -m pip install --user virtualenv```

    For Windows:
    ```py -m pip install --user virtualenv```

    Then create a new environment using the command:

    For MacOs and Linux:
    ```python3 -m venv env```

    For Windows:
    ```py -m venv env```

    venv will create a virtual Python installation in the `env` folder.

    Activate the environment:

    For MacOs and Linux:
    ```source env/bin/activate```

    For Windows:
    ```.\env\Scripts\activate```

    Confirm that your environment was correctly installed and it is active with the command:

    For MacOS and Linux:
    ```
    which python
    .../env/bin/python
    ```

    For Windows:
    ```
    where python
    .../env/bin/python.exe
    ```

  * Now that you have your venvironment up and it is activated install all the project dependencies on it.

    Go to the path `webots-project/controllers/pos-prediction` and run the command `pip install -r requirements.txt`.
    This command will install all the project dependencies which are in the `requirements.txt` file.

5. Go to the folder `webots-project/worlds` you will see two worlds:
  * `pos-prediction-user.wbt`: This world has configured to run the robot controller inside the Webots tool.
  * `pos-prediction-dev.wbt`: This world has configured to run the robot controller outside the Webots tool using an IDE or Python Command lines, etc. 

  In this section the file called `pos-prediction-user.wbt` will be used. Open it with a double click should lunch the Webots application and the world will be load into it. 

6. In Webots open the preferences window under the `Webots/Preferences...` option and configure the Python command to be the python executable which is in the recently created virtual environment in the path: `webots-project/controllers/pos-prediction/env/bin/python3`

![Preferences](https://github.com/joangerard/webots-thesis/img/preferences.png "Preferences Configuration Webots")

7. Open the Robot Window: click on the robot then right click on it and select the option Show Window Robot. The robot window will be displayed in the left side of the screen.

![Show Robot Window](https://github.com/joangerard/webots-thesis/img/robot-window.png "Show Robot Window")



