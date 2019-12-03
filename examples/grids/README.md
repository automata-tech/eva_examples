# Grid Example

This example code shows how a user might go about implementing their own 2D grid system. It provides a 2D grid class, which provides a series of grod positions with input of:

- 3 XY corner positions
- A number of rows
- A number of columns

Using this 2D grid as a building block, you will be able to create more complex interactions, for example picking from one grid and placing into another!

## Compatibility

Requirements to run this script as is:

- Eva software version: 3.x.x
- Eva Python SDK: 2.x.x

## Setting up and running

This project has used pipenv for dependency mamangement, please ensure you have Python3 and pipenv installed.

    # Download the example code and enter navigate the terminal to this directory
    $ pipenv install

    # Caution, this will cause Eva to move!
    $ pipenv run main.py

## Project Description

### grid2d.py

This file contains the Grid2D class, given grid corners and rows and columns, it will output a series of grid positions.

### main.py

This contains logic to connect to an Eva, makes a Grid2D and then moves the robot to each point in the grid.
This is a good starting place if you are looking to create your own custom grid logic.

## Implementation Notes

This example uses a series of goto's and calculates the joint angles for each grid position between each goto.
This is for the sake of keeping the example as simple as possible but is slow as the inverse kinamatics calculation takes time.
To speed up the operation, either use a toolpath or calculate the inverse kinamatics prior to or during movement.
