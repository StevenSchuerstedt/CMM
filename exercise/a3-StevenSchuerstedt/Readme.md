# Assignment 3 - Boids

## Integration Schemes
I added a menu to switch between the three different integration schemes. 
Midpoint seems to work the best, but it requires the most computations.

## Flocking
Demonstration of Cohesion, Separation an Alignment in flocking.mp4. 
Also I added a mode with all the three forces combined (mode ALL in the video).
Every behaviour is modeled with a force, and all forces are summed up. The different integration schemes then apply these forces to update the position and velocity of the boids.

## Leader Boid
Demonstation of leader behaviour and collision in leader.mp4. The big green boid is the leader. Other boids try to follow the leader and the leader is controlled with the mouse. Collision is done by adding a repelling force when boids get to close to the blue circle.  
![img1](https://user-images.githubusercontent.com/50596774/114852376-ff84e000-9de2-11eb-9d2b-3e6f6e537229.png)

## Collaborative and Adversarial Behaviors
I divided the boids in two groups, red and blue, and colored them accordingly. Also I introduced a counter to display the total number of boids and the number of red and blue boids. Boids are added / removed according to the laws described in the exercise sheet. In order for the simulation to not blow up, I limit the adding of boids with a time constraint. Then I montiored the boids behaviour with the ALL mode (so cohesion, separation and alignment forces activated). The evolution of the boids heavily depends on the concrete values one uses for the difference forces. When the forces are unbalanced, so cohesion is much stronger than separation for example, one group of boids will dominate the other and will eventually rule out the ohter group completly. With a more balanced approach I managed to get a stable growing number of boids living happily next to each other.

#Red / #BLUE = Ratio

130 / 110 = 1.18

160 / 138 = 1.15

215 / 165 = 1.30

260 / 198 = 1.31

312 / 240 = 1.3

343 / 264 = 1.29

374 / 301 = 1.24

Both group of boids are growing, but they maintain the same ratio throughout. (inital number of boids: n = 400)
One example of unbalanced / balanced forces can be seen in group.mp4. 

![img2](https://user-images.githubusercontent.com/50596774/115003941-936bb000-9ea6-11eb-95bb-d8f7c76cffd7.PNG)
 
### Control Strategy
A simple control strategy to increase the size of a boid population is, that boids of the same group are attracted to each other. More precisely there is a force in the direction of the closest boid in the same group. By applying this control strategy to only one group, the group keeps growing steadily, but without making the other group go extinct completly. The boids in the other group are not "attacked" so they can stay as they are. 
When applying the control strategy to both groups, they are bot rising in number almost equaly. They form groups of homogenous boids sticking to each other.
