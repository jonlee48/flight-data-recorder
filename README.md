# flight-data-recorder

## Table of Contents
- [Getting Started](#getting-started)
- [Installing Miniconda](#installing-miniconda)
- [Setting up the Environment](#setting-up-the-environment)
- [Running the Notebook](#running-the-notebook)
- [Visualizing Data](#)


## Getting Started
If you want to run the code, interact, and play around with the data, you'll want to clone this repo and get python set up on your computer. The installation below walks through setting up Miniconda and the conda environment. If you already have Anaconda installed, you can skip to [Setting up the Environment](#setting-up-the-environment). All the data processing and visualization code is in a single notebook: `sensor/data_analysis.ipynb`. All the raw flight data is stored in one directory: `sensor/inputData/`.

The code for the Feather is in the `feather` directory.

## Installing Anaconda
Install [Anaconda](https://www.anaconda.com/products/individual) for your OS. If you're familiar with using the commandline, then use Anaconda Prompt. If you'd rather use a graphical interface, then follow the steps for the Anaconda Navigator.

Anaconda Prompt:
```
git clone https://github.com/jonlee48/flight-data-recorder.git
cd flight-data-recorder
conda deactivate
conda env create -f my_env_no_builds.yml
conda env list
conda activate fdr
```

Using Anaconda Navigator:
1. Download and unzip this repository. 
2. Select Environment, Import. Name it 'fdr', and select the `my_env.yml` from your downloads.\
3. Import


## Installing Miniconda
Miniconda is a lightweight distribution of Anaconda, a python distribution. Installing either Miniconda or Anaconda will install python, some popular data science packages, and conda. Conda is the package and environment manager. It helps manage and isolate packages for your different python projects. You use it with command line commands in the terminal or with the Anaconda Prompt. I'm using Miniconda over Anaconda because we don't need all the bulky packages that Anaconda installs. Miniconda gives us a minimal installation and we can just install a few packages we need.

Skip this step if you already have Anaconda or Miniconda installed. You can check by running `conda` in a terminal. 

If you're using Windows, you can either install Anaconda from the browser, and use Anaconda Prompt, or you can use WSL2. ([Here](https://towardsdatascience.com/configuring-jupyter-notebook-in-windows-subsystem-linux-wsl2-c757893e9d69) is how to install WSL2). Then follow the instructions below. If you're using Mac or Linux, run these commands in shell.
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

## Setting up the Environment
Now we'll use conda to set up a new python environment. Run `conda env list` to see a list of environments. You should see the `base` environment, the default environment. We're going to make a new one for this project called `sensor`. First, clone this repo and `cd` into it.
```
git clone https://github.com/jonlee48/flight-data-recorder.git
cd flight-data-recorder
conda env create -f environment.yml
conda activate sensor
conda env list
```
We have now created a new environment built from the `environment.yml` yaml file. You should now see `$(sensor)` to the left of your prompt. This is the name of the environment you just created. It is now active, so any python code executed in that shell will run in the `sensor` environment.

Here's a reference of some useful conda commands to keep handy for later. See the [docs](http://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html
) for more info.
```
conda env list
conda create -n env_name
conda activate env_name
conda deactivate
conda env export > environment.yml
conda env create -f environment.yml
conda remove --name myenv --all
```

## Running the Notebook
Before we start the Jupyter lab server, we want to setup interactive widgets. This allows us to get interactive plots in the notebook!
```
jupyter labextension install @jupyter-widgets/jupyterlab-manager
jupyter lab build
```
Now we can finally run the notebook!
```
jupyter lab --no-browser
```
