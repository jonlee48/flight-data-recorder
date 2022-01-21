# flight-data-recorder

## Table of Contents
- [flight-data-recorder](#flight-data-recorder)
  - [Table of Contents](#table-of-contents)
  - [About](#about)
  - [Setup](#setup)
    - [Arduino](#arduino)
  - [Data Analysis and Visualization](#data-analysis-and-visualization)
    - [Anaconda](#anaconda)
    - [Running the Jupyter Notebooks](#running-the-jupyter-notebooks)
    - [Running the Web Dashboard](#running-the-web-dashboard)
  - [Contributing](#contributing)

## About
The Flight Data Recorder (FDR) is a microcontroller based system designed to collect flight performance metrics for the George Washington University Design Build Fly Club. Data obtained from various onboard sensors (altimeter, pitot tube, IMU, and GPS) is visualized on a real-time Ground Station and analyzed post flight. This information provides valuable feedback for the design team by confirming flight characteristics and identifying areas of improvement for the next design iteration.

## Setup
Download this repository and unzip or run `git clone https://github.com/jonlee48/flight-data-recorder` in the terminal.

### Arduino
To run the Arduino sketches, download the Arduino IDE and set the Sketchbook Location to `/path/to/flight-data-recorder/Arduino`. You will also need to follow the [setup instructions](https://learn.adafruit.com/adafruit-feather-m0-adalogger/setup) for using the Arduino IDE with the Adafruit Feather. 

The FDR and Ground Station share identical hardware, but the software is different. The FDR runs the `Arduino/dbf2022/fdrv7/fdrv7.ino` program. And the Ground Station runs the `Arduino/dbf2022/groundstation/groundstation.ino` program. There are also test scripts in the `Arduino` directory organized by year.

## Data Analysis and Visualization
A live web dashboard is run on a field laptop connected to the Ground Station. The Ground Station continuously receives new data transmitted from the FDR over Long Range (LoRa) radio. The FDR also records a more detailed log on an onboard microSD card which is processed post-flight. 

Install Anaconda to run the visualization tools.

### Anaconda
Anaconda will install python, some popular data science packages, and `conda` the package manager. Conda helps you manage packages and environments for different python projects. Once installed, open Anaconda Navigator and setup a new python environment. Select Environments and Import. Choose `my_env.yml` from this repo as the Specification File. The name and location will autofill, so just click Import.

From the Anaconda Navigator you can launch an IDE of your choice (e.g. PyCharm, VSCode, Spyder) or other applications, such as Jupyter Lab/Notebook. The application will run with the selected python environment. You can also manage packages from the Anaconda Navigator. 

### Running the Jupyter Notebooks
Launch JupyterLab from the Anaconda Navigator. Make sure you are running the `conda_env_fdr` environment and not the `base` environment. Once the Jupyter server is running, you can open up the notebooks in the `src` directory. All the notebooks are organized by year. The `src/../flight_report.ipynb` notebook is used to generate a flight report from the logs collected on the microSD card. 

### Running the Web Dashboard
Launch the IDE of your choice from the Anaconda Navigator (PyCharm, VSCode, and Spyder all work). From the IDE run the `src/src2022/server.py` script or run `python server.py` in the terminal (with environment activated). The dashboard will be served locally at [`127.0.0.1:7000`](127.0.0.1:7000).

## Contributing
Minor bug fixes and updates should be committed directly to the main branch. Larger features (like a new widget on the dashboard) should be made in a separate branch. Once you're satisfied with your changes make a pull request and I will merge it into the main branch. This enables me to review your changes first, before it is released into the main codebase.

A list of to do items are found under Projects tab > DBF2022. Any of the cards in the 'To Do' column are fair game (or you can add a card of your own). Pick a card you like and move it to the 'In Progress' column to claim it. 