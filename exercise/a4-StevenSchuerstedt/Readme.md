First Name: Timothy Steven

Last Name: Schürstädt

Solution to Question 4:

<a href="https://www.codecogs.com/eqnedit.php?latex=tr(F^TF&space;-&space;I)&space;=||F||_F^2&space;-&space;2&space;\newline&space;F&space;=&space;\begin{pmatrix}&space;a&space;&&space;b&space;\\&space;c&space;&&space;d&space;\end{pmatrix}&space;\newline&space;tr(F&space;*&space;F^T&space;-&space;I)&space;=&space;tr(\begin{pmatrix}&space;a^2&space;&plus;c^2-&space;1&space;&&space;ab&space;&plus;&space;cd&space;\\&space;ab&plus;cd&space;&&space;d^2&space;&plus;&space;b^2&space;-&space;1&space;\end{pmatrix})&space;=&space;a^2&space;&plus;&space;b^2&space;&plus;&space;c^2&space;&plus;&space;d^2&space;-&space;2&space;=&space;(\sqrt{a^2&space;&plus;&space;b^2&space;&plus;&space;c^2&space;&plus;&space;d^2})^2&space;-2&space;=&space;||F||_F^2&space;-&space;2" target="_blank"><img src="https://latex.codecogs.com/gif.latex?tr(F^TF&space;-&space;I)&space;=||F||_F^2&space;-&space;2&space;\newline&space;F&space;=&space;\begin{pmatrix}&space;a&space;&&space;b&space;\\&space;c&space;&&space;d&space;\end{pmatrix}&space;\newline&space;tr(F&space;*&space;F^T&space;-&space;I)&space;=&space;tr(\begin{pmatrix}&space;a^2&space;&plus;c^2-&space;1&space;&&space;ab&space;&plus;&space;cd&space;\\&space;ab&plus;cd&space;&&space;d^2&space;&plus;&space;b^2&space;-&space;1&space;\end{pmatrix})&space;=&space;a^2&space;&plus;&space;b^2&space;&plus;&space;c^2&space;&plus;&space;d^2&space;-&space;2&space;=&space;(\sqrt{a^2&space;&plus;&space;b^2&space;&plus;&space;c^2&space;&plus;&space;d^2})^2&space;-2&space;=&space;||F||_F^2&space;-&space;2" title="tr(F^TF - I) =||F||_F^2 - 2 \newline F = \begin{pmatrix} a & b \\ c & d \end{pmatrix} \newline tr(F * F^T - I) = tr(\begin{pmatrix} a^2 +c^2- 1 & ab + cd \\ ab+cd & d^2 + b^2 - 1 \end{pmatrix}) = a^2 + b^2 + c^2 + d^2 - 2 = (\sqrt{a^2 + b^2 + c^2 + d^2})^2 -2 = ||F||_F^2 - 2" /></a>

Solution to Question 10:
The optimizer only takes the position into account and does not optimize according to triangle energy, so no rotation of handles.

Solution to Question 11:
The optimizer should also minimize the energy of the triangle, so the regularizer adds the medium of the energy of all triangles. 

---

Assignment writeup: http://crl.ethz.ch/teaching/computational-motion-21/slides/tutorial-a4.pdf

---

Could use ./build.sh on Linux/MacOS
