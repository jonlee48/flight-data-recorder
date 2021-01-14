# flight-data-recorder


## Table of Contents
- [Getting Started](#-getting-started)
- [How to run notebooks](#-how-to-run-notebooks)
- [Inputting Data](#)
- [Visualizing Data](#)


## Getting Started
If you want to run the code, interact, and play around with the data, you'll want to clone this repo and get python set up on your computer. The items of note is the data processing and visualization notebook which can be found in `sensor/data_analysis.ipynb`. The raw flight data belongs in the`sensor/inputData/` directory as text files in CSV format.

## Installing Miniconda
Miniconda is a lightweight distribution of Anaconda, a python distribution. Installing either Miniconda or Anaconda will install python, some popular data science packages, and conda. Conda is the package and environment manager. It helps manage and isolate packages for your different python projects. You use it with command line commands in the terminal or with the Anaconda Prompt. I'm using Miniconda over Anaconda because we don't need all the bulky packages that Anaconda installs. Miniconda gives us a minimal installation and we can just install a few packages we need.

Skip this step if you already have Anaconda or Miniconda installed.

If you're using Windows, you can either install Anaconda from the browser, and use Anaconda Prompt, or you can use WSL2. ([Here](https://towardsdatascience.com/configuring-jupyter-notebook-in-windows-subsystem-linux-wsl2-c757893e9d69) is how to install WSL2).  If you're using Mac or Linux, run these commands in shell.
```
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
chmod +x Miniconda3-latest-Linux-x86_64.sh
sh Miniconda3-latest-Linux-x86_64.sh
rm Miniconda3-latest-Linux-x86_64.sh
```
Now that Miniconda is installed, here are some commands I like to run.
```
conda config --set auto_activate_base false
conda update conda
conda update --all
```
Now run `conda env list`. You should see the `base` environment. We're going to make a new one for this project and its dependencies. Now `cd` into the `flight-data-recorder/` directory and run:
```
conda env create -f environment.yml
conda activate sensor
conda env list
```
This creates a new environment from the `environment.yml` file which specifies all the dependencies and versions to install.
Now you should `$(sensor)` to the left of your prompt. This is the name of the environment you just created. It is now active, so any python executed in that shell will run in the `sensor` environment.

Here's a reference of some useful commands to keep handy for later. See the [docs](http://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html
) for more info.
```
conda env list
conda create -n env_name
conda activate env_name
conda deactivate
conda env export > environment.yml
conda env create -f environment.yml
```

## Run Jupyter
Now to open the notebook, just run
```
jupyter lab --no-browser
```
