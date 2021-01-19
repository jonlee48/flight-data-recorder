# flight-data-recorder

## Table of Contents
- [Getting Started](#getting-started)
- [Installing Anaconda](#installing-anaconda)
- [Setting up the Environment](#setting-up-the-environment)
- [Running the Notebook](#running-the-notebook)
- [Visualizing Data](#)


## Getting Started
If you want to run the code, interact, and play around with the data, you'll want to clone this repo and get python set up on your computer. The installation below walks through setting up Miniconda and the conda environment. If you already have Anaconda installed, you can skip to [Setting up the Environment](#setting-up-the-environment). All the data processing and visualization code is in the`src/` directory. To open the interactive flight report, follow the instructions to install Anaconda and Setting up the Environment. Then open `src/flight_report.ipynb`. All the raw flight data is in the `src/inputData(mm-dd-yy)/` directories.

The code for the Adafruit Feather is in the `Arduino` directory.

## Installing Anaconda
Installing Anaconda will install python, some popular data science packages, and conda. Conda is a python package and environment manager. It helps manage and isolate packages for your different python projects. You can use it with either Anaconda Prompt (a terminal) or Anaconda Navigator (a GUI).

> Note: you can also install Miniconda, which is a lightweight distribution of Anaconda, so it takes up less disk space. [Here](https://towardsdatascience.com/configuring-jupyter-notebook-in-windows-subsystem-linux-wsl2-c757893e9d69) is how to install Miniconda for WSL2 or Linux. Essentially, you'll want to run these commands in terminal.
> ```
> cd ~
> wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
> chmod +x Miniconda3-latest-Linux-x86_64.sh
> sh Miniconda3-latest-Linux-x86_64.sh
> rm Miniconda3-latest-Linux-x86_64.sh
> ```

Install Anaconda from their [website](https://www.anaconda.com/products/individual) for your OS. Follow the default installation instructions. 


## Setting up the Environment
Now, we'll need to setup the packages and dependencies to run the code in an isolated environment. If you're familiar with using the commandline, then use Anaconda Prompt. Otherwise, use Anaconda Navigator with its graphical interface.

### Anaconda Prompt:
Type `conda env list` to see a list of environments. You should see the `base` environment, the default environment. We're going to make a new one for this project called `fdr`.
```
git clone https://github.com/jonlee48/flight-data-recorder.git
cd flight-data-recorder
conda deactivate
conda env create -f my_env_no_builds.yml
conda env list
conda activate fdr
```
We have now created a new environment built from the yaml file. You should now see `$(fdr)` to the left of your prompt. This is the name of the environment you just created. It is now active, so any python code executed in that shell will run in the `fdr` environment.

### Anaconda Navigator:
1. Download and unzip this repository. 
2. Select Environment, Import. Name it 'fdr', and select the `my_env.yml` from your downloads.\
3. Click Import

### Next
Now that conda is installed, here are some commands I like to run.
```
conda config --set auto_activate_base false
conda update conda
conda update --all
```

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
Now it's time to run the notebook.

If you're using Anaconda Prompt:
```
jupyter lab --no-browser
```
If there are any issues seeing the plots, you might have to shutdown the notebook and setup interactive widgets.
```
jupyter labextension install @jupyter-widgets/jupyterlab-manager
jupyter lab build
```

If you're using Anaconda Navigator, click the environment name and the green run button. Select 'Open with Jupyter Notebook'.
