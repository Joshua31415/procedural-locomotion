# procedural-locomotion

Code base for the project: Procedural Locomotion Engine For Digital Humans.

## Getting started

We recommend [JetBrains CLion](https://www.jetbrains.com/clion/) for the development.
It is a paid software, but JetBrains has the [student plan](https://www.jetbrains.com/community/education/#students)
that provides free licenses. See [this](https://www.jetbrains.com/help/clion/clion-quick-start-guide.html) for a quick
start guide.

1. Fork this repository and download the code.

2. Build the project (or build `locoApp`). You can build the project in cmake Release mode for realtime performance:
   see [this](https://www.jetbrains.com/help/clion/cmake-profile.html) for a guide about cmake profile for CLion.

3. Run the `locoApp`.

4. Select a model to play with: `Main Menu > Character > Model`. We have `Bob` and `Running Bob` for examples.

![Screen](img/screen.png)

5. Press the space bar to play the app. You can give joystick command with arrow keys in your keyboard. 


## Results

Walking in a straight line
![Walking straight](img/walking_straight.gif)


Walking along a curved trajectory
![Walking curved](img/walking_curve.gif)

Running
![Running](img/running.gif)

Walking on uneven terrain
![Running](img/walking_uneven.gif)


