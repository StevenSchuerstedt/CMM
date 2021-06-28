# Assignment 6 - Trajectory Optimization

**Hand-in:** May 28, 2021, 18:00 CEST 

First Name: Timothy Steven

Last Name: Schürstädt

Solution to Question 2:
a_k = 1/h * ((x_k+2 - x_k+1) - (x_k+1 - x_k))

Solution to Question 5:
The objective function is very close to a quadratic so newtons method converges really fast. Newtons Method uses the gradient and the hessian to construct a quadratic function.

Solution to Question 7:
![Image description](https://imgur.com/a/KsV7cbd)

Solution to Question 9:
The pysics constraint dominate the gradient so it never moves in the other directions to minimize the function. It could work with a really small stepsize for gradient descent but that would take forever to converge. 

Explanation of your approach to Question 10 (required for full credit):
- distance of spaceship to sun should equal radius
- the speed should be so fast the centripetal force neglects gravity
- the direction of the speed vector should be tangential on the circle, so dot product with r vector is 0

---

Assignment writeup: http://crl.ethz.ch/teaching/computational-motion-21/slides/Tutorial-Write-Up-6.pdf

---

- NOTE: Don't forget to switch to Release mode or Release With Debug Info otherwise it's gonna be just unimaginably slow.
- NOTE: Tested builds: GCC 9.3 (Linux), Visual Studio 2019 (Windows), Clang 12.0.0 (Mac OS)
