# AMR_Project

Simulator: Webots: robot simulator
Language: Python

Steps to replicate the project:
  1. In Webots create a new project directory by going to 'Wizards' and selecting 'New Project Directory'
  2. In world settings select 'Add a rectangular arena' option and continue
  3. Change the dimensions of the 'Rectangular Arena' from the left menu and increase the size to 5x5
  4. Change the 'coordinateSystem' in 'WorldInfo' to "NUE" for using GPS
  5. Import the custom Turtlebot3 robot in the repository
  6. In Webots window go to 'Wizards' and select 'New Robot Controller'
  7. Create a new controller in python language.
  8. Open the controller in Text editor and copy the controller code in AMR_Project/wall_follower/controllers/AMR_Project_G7_robot_controller to the 'New Robot Controller'
  9. Select the new controller by going to robots dropdown menu and selecting the controller tab.
  10. Save the world and code. 
  11. Select the 'ReatcngularArena' and add obstacles in the arena.
  12. Run the program.

Path and waypoint
![Waypoints and path](https://user-images.githubusercontent.com/67186743/145256554-c7b2b788-b2cc-4c06-9c72-147cd725b342.jpg)

Video
https://user-images.githubusercontent.com/67186743/145256630-14da3cae-e6fb-4a05-bee0-a9071f64c7a3.mp4
