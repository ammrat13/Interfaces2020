# GT IEEE Robotics: Interfaces 2020
This interface package defines high level abstractions for low level request and generalizes these abilities to multiple systems. For example, reading an image is abstracted into a simple call and has been generalized to Raspberry Pi, Jetson Nano, and a simulated camera. For ease of development, the simulated interface allows us to interact with the simulated robot without actual hardware. See the Simulator2020 repo for info on the sim.

## How to Run

If you don't have a conda environment, first install miniconda, then:
```
$ conda create --name southeastcon2020 PYTHON=3.7.1
$ conda activate southeastcon2020
```
Now clone and visit the `Simulator2020` repo, and run the following.
```
$ pip install -e .
```
Then come back here and run it in this repo.

## Example Script

```
import interface

interface.set_system("sim")
print(interface.read_image().shape)
print(interface.time())
```
