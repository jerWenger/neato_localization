# Computational Robotics Localization

You should include a couple of bag files of your code in action. Place the bag files in a subdirectory of your ROS package called “bags”. In this folder, create a README file that explains each of the bag files (how they were collected, what you take from the results, etc.).

Writeup (Due 10-12)

In your ROS package create a README.md file to hold your project writeup. Your writeup should touch on the following topics. We expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio. As such, please make sure to write the report so that it is understandable to an external audience. Make sure to add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool peek).

# Goals
Our goals for this project were the following:
- Use the starter code to implement a clean example of a particle filter
- Be cautious of computational cost and think of ways to optimize for speed
- Think of the possible applications this algorithm could have in our research


# How did you solve the problem? 
How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).

# Design Decision
The largest design decision we faced was deciding how we wanted to handle our reference frames and the relationships between them. There are two main options here that ultimately achieve the same goal: using basic trig functions to establish relationships between coordinates and their frames or take it a step further and implement translation and rotation matrices.

We ultimately decided that the matrix approach would be the best way to go. This is mostly a “future proofing” decision. While right now it may seem like hitting a needle with a sledge hammer, if we were to move into the three dimensional space with our robot it would become a necessity.

$$inv(\begin{bmatrix} cos(theta_1) & -sin(theta_1) & X_1 \\ sin(theta_1) & cos(theta_1) & Y_1 \\ 0 & 0 & 1 \end{bmatrix}) \begin{bmatrix} cos(theta_2) & -sin(theta_2) & X_2 \\ sin(theta_2) & cos(theta_2) & Y_2 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} cos(theta_p) & -sin(theta_p) & X_p \\ sin(theta_p) & cos(theta_p) & Y_p \\ 0 & 0 & 1 \end{bmatrix}$$ 
Where subscript 1 is the reference frame of the robots old position, subscript 2 is the reference frame of the robots current position, and p is the reference frame of the particle.

Another important decision we made was how to distribute particles when the filter is initialized. We decided that the best way to do this would be to evenly distribute particles around our initial location in an equalized grid form. 

This allows for an easy distribution that we know will be resistant to possible biases or inconsistencies that you could possibly see in a random initialization. 


# Challenges
What if any challenges did you face along the way?

# Improve if we had more time
What would you do to improve your project if you had more time?

# Interesting lessons
Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

# Bag file explanation
