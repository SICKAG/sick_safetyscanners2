^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_safetyscanners2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2024-09-24)
------------------
* possible fix for out of range
* Add material for correct representation in Gazebo Sim.
* Enable workin in Gazebo under humble.
* enabled gazebo integration in urdf
* generated description folder using RTW
* diagnostics for lifecycle node aswell
* refactor: combine Node and LifeCycle node implementations
* Contributors: Dr. Denis Štogl, Lennart Puck, Nibanovic, Rein Appeldoorn

1.0.3 (2021-12-22)
------------------
* Fixes unsafe pointer access in UDP callback
* Implement lifecycle node 
* Added functionality to allow multicast
* set not using the default sick angles as default
* moved changeSensor settings to be always be invoked
* fixed typo in launch file
* Contributors: Brice, Erwin Lejeune, Soma Gallai, Lennart Puck, Tanmay

1.0.2 (2021-03-15)
------------------
* added missing dependencies to package xml
* Contributors: Lennart Puck

1.0.1 (2021-03-05)
------------------
* changed the parameter callback interface so its only triggered
  when the parameters of this node are called
* Contributors: Lennart Puck

1.0.0 (2021-01-11)
------------------

* Initial Release
